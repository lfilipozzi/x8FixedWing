classdef Target < matlab.System & ...
        matlab.system.mixin.SampleTime & ...
        matlab.system.mixin.Propagates
% Target Generate target by solving the following quadratic
% optimal programming problem.
%   min ||u - Ka*ua||_Q^2 + ||Delta ua||_R^2
%   subject to:
%       actuator bounds
%       (soft) polytopic constraints

properties(Nontunable,Logical)
    % Actuator bounds
    hasActuatorBounds = false;
    % Inequality constraints
    hasIneqConstraints = false;
    % Soft inequality constraints
    hasSoftConstraints = false;
    % Show solving time
    showSolveTime = false;
    % Show slack variable
    showSlackVariable = false;
    % Show exitflag
    showExitflag = false;
end

properties(Nontunable)
    % Sampling time
    Tsampling = 0.01;
    % Map to generalized controls
    Ka = zeros(4,6);
    % Tracking penalty
    Qmat = eye(4);
    % Rate of change penalty
    Rmat = eye(6);
    % Indices
    softConstraintsIndex = {0};
    % Weight
    softConstraintsWeight = 1;
end

properties(DiscreteState)
    
end

% Pre-computed constants
properties(Access = private)
    quadprog_option % Options for the quadprog solver
    Nu              % Number of generalized controls
    NuAlpha         % Number of allocated controls
end

methods(Access = protected)
    function setupImpl(obj)
        % Perform one-time calculations, such as computing constants

        % Set quadprog options
        obj.quadprog_option = ...
            optimoptions('quadprog','display','off');

        % Number of generalize and allocated controls
        obj.Nu      = size(obj.Ka,1);
        obj.NuAlpha = size(obj.Ka,2);
    end

    function varargout = stepImpl(obj, varargin)
        % Compute optimal braking torque distribution among actuators
        
        num = 2;
        ref = varargin{num-1};
        uAlphaPrev = varargin{num};
        if obj.hasActuatorBounds
            num = num + 2;
            umin = varargin{num-1};
            umax = varargin{num};
        else
            umin = [];
            umax = [];
        end
        if obj.hasIneqConstraints
            num = num + 2;
            A = varargin{num-1};
            b = varargin{num};
        else
            A = [];
            b = [];
        end

        % Setup chronometer
        tic;

        % Allocate the controls
        [u,exitflag] = obj.allocate(ref,uAlphaPrev,umin,umax,A,b);

        % Return output
        time = toc;
        uOut = u(1:obj.NuAlpha);
        
        num = 1;
        varargout{num} = uOut;
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = time;
        end
        if obj.showExitflag
            num = num + 1;
            varargout{num} = exitflag;
        end
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            Ns = obj.getNumberSlackVar();
            slack = u(obj.NuAlpha+(1:Ns));
            varargout{num} = slack;
        end
    end

    function resetImpl(obj)
        % Reinitialize discrete states
        
    end

    function releaseImpl(~)
        % Release resources, such as file handles
        
    end

    function validatePropertiesImpl(obj)
        % Validate related or interdependent property values

        obj.setupImpl();

        % Check Q matrix size
        if size(obj.Qmat,1) ~= obj.Nu || size(obj.Qmat,2) ~= obj.Nu
            msg = 'The size of the Q matrix is not valid';
            error(msg);
        end

        % Check R matrix size
        if size(obj.Rmat,1) ~= obj.NuAlpha || ...
                size(obj.Rmat,2) ~= obj.NuAlpha
            msg = 'The size of the R matrix is not valid';
            error(msg);
        end
    end

    function flag = isInactivePropertyImpl(obj,prop)
        % Return false if property is visible based on object 
        % configuration, for the command line and System block dialog
        if ismember(prop, {'hasSoftConstraints'})
            flag = not(obj.hasIneqConstraints);
        elseif ismember(prop, {'softConstraintsIndex',...
                'softConstraintsWeight','showSlackVariable'})
            flag = not(obj.hasIneqConstraints && obj.hasSoftConstraints);
        else
            flag = false;
        end
    end
end

methods(Access = private)
    function [uOut, exitflag] = allocate(obj, ref, uAlphaPrev, ulb, uub, A, b)
        % Allocate the controls

        u = ref;                    % Generalized controls
        K = obj.Ka;

        Q  = obj.Qmat;
        R  = obj.Rmat;
        Ts = obj.Tsampling;

        H =  2 * (K'*Q*K + 1/Ts^2 * R);
        f = -2 * (K'*Q*u + 1/Ts^2 * R*uAlphaPrev);

        % Equality constraints
        Aeq = [];
        beq = [];
        
        % Soft constraints
        if obj.hasIneqConstraints && obj.hasSoftConstraints
            Ns = obj.getNumberSlackVar();
            As = zeros(size(A,1),Ns);
            for i = 1:Ns
                As(obj.softConstraintsIndex{i},i) = -1;
            end
            Hs = diag(obj.softConstraintsWeight);
            % Modify optimization problem
            H = blkdiag(H,Hs);
            f = [f; zeros(Ns,1)];
            A = [A As];
            ulb = [ulb; zeros(Ns,1)];
            uub = [uub; inf(Ns,1)];
        end

        [uOut,~,exitflag] = quadprog(...
            H,f,A,b,Aeq,beq,ulb,uub,[],obj.quadprog_option...
            );

        % Warning if the problem has not been solved
        if exitflag < 1
            errMessage = obj.getWarningMessage(exitflag);
            warning(errMessage);
        end
        
        if isempty(uOut)
            error('Solver failed! No solution.'); 
        end
    end

    function errMessage = getWarningMessage(~,exitflag)
        errMessage = 'Solver failed!';
        switch exitflag
            case 0
                errMessage = [errMessage, ...
                    ' Maximum number of iterations exceeded.'];
            case -2
                errMessage = [errMessage, ...
                    ' Problem is infeasible.'];
            case -3
                errMessage = [errMessage, ...
                    ' Problem is unbounded.'];
            case -6
                errMessage = [errMessage, ...
                    ' Nonconvex problem detected.'];
            case -8
                errMessage = [errMessage, ...
                    ' Unable to compute a step direction.'];
            otherwise
                errMessage = [errMessage, ' exitflag is ', ...
                    num2str(exitflag)];
        end
    end
    
    function Ns = getNumberSlackVar(obj)
        % Return the number of slack variables
        if obj.hasIneqConstraints && obj.hasSoftConstraints
            Ns = numel(obj.softConstraintsIndex);
        else
            Ns = 0;
        end
    end
end

methods(Access = protected)
    function sts = getSampleTimeImpl(obj)
        % Define sampling time
        sts = createSampleTime(obj,'Type','Discrete',...
          'SampleTime',obj.Tsampling,'OffsetTime',0);
    end

    function ds = getDiscreteStateImpl(~)
        % Return structure of properties with DiscreteState attribute
        ds = [];
    end

    function num = getNumInputsImpl(obj)
        % Define total number of inputs for system with optional inputs
        num = 2;
        if obj.hasActuatorBounds
            num = num + 2;
        end
        if obj.hasIneqConstraints
            num = num + 2;
        end
    end

    function num = getNumOutputsImpl(obj)
        % Define total number of outputs for system with optional
        % outputs
        num = 1;
        if obj.showSolveTime
            num = num + 1;
        end
        if obj.showExitflag
            num = num + 1;
        end
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
        end
    end

    function varargout = getInputNamesImpl(obj)
        % Return input port names for System block
        num = 2;
        varargout{num-1} = 'gen. controls';
        varargout{num} = 'prev. controls';
        if obj.hasActuatorBounds
            num = num + 2;
            varargout{num-1} = 'min. actuator';
            varargout{num}   = 'max. actuator';
        end
        if obj.hasIneqConstraints
            num = num + 2;
            varargout{num-1} = 'const. matrix';
            varargout{num}   = 'const. bound';
        end
    end

    function varargout = getOutputNamesImpl(obj)
        % Return output port names for System block
        num = 1;
        varargout{num} = 'alloc. controls';
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = 'time';
        end
        if obj.showExitflag
            num = num + 1;
            varargout{num} = 'exitflag';
        end
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            varargout{num} = 'slack var.';
        end
    end

    function flag = isInputSizeLockedImpl(~,~)
        % Return true if input size is not allowed to change while
        % system is running
        flag = true;
    end

    function varargout = isOutputFixedSizeImpl(obj)
        % Return true if output size is not allowed to change while
        % system is running
        
        % Allocated controls
        num = 1;
        varargout{num} = true;
        % Solving time
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = true;
        end
        % Exitflag
        if obj.showExitflag
            num = num + 1;
            varargout{num} = true;
        end
        % Slack variable
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            varargout{num} = true;
        end
   end

    function varargout = getOutputSizeImpl(obj)
        obj.setupImpl();
        % Return size for each output port
        
        % Allocated controls
        num = 1;
        varargout{num} = [obj.NuAlpha, 1];
        % Solving time
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = [1, 1];
        end
        % Exitflag
        if obj.showExitflag
            num = num + 1;
            varargout{num} = [1, 1];
        end
        % Slack variable
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            Ns = obj.getNumberSlackVar();
            varargout{num} = [Ns, 1];
        end
    end

    function varargout = getOutputDataTypeImpl(obj)
        % Allocated controls
        num = 1;
        varargout{num} = 'double';
        % Solving time
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = 'double';
        end
        % Exitflag
        if obj.showExitflag
            num = num + 1;
            varargout{num} = 'double';
        end
        % Slack variable
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            varargout{num} = 'double';
        end
    end

    function varargout = isOutputComplexImpl(obj)
        % Allocated controls
        num = 1;
        varargout{num} = false;
        % Solving time
        if obj.showSolveTime
            num = num + 1;
            varargout{num} = false;
        end
        % Exitflag
        if obj.showExitflag
            num = num + 1;
            varargout{num} = false;
        end
        % Slack variable
        if obj.hasIneqConstraints && obj.hasSoftConstraints && ...
                obj.showSlackVariable
            num = num + 1;
            varargout{num} = false;
        end
    end

end

methods(Access = protected, Static)
    function header = getHeaderImpl
        header = matlab.system.display.Header('Target',...
            'Title','Target generation',...
            'Text', ['Generate targets by solving an optimal', ...
            'quadratic programming problem.']);
    end

    function group = getPropertyGroupsImpl
        % Customize dialog box
        
        % +-----------------------+
        % |      Formulation      |
        % +-----------------------+
        % Create section for model
        MatricesSection = matlab.system.display.Section(...
            'Title','Weight matrices',...
            'PropertyList',{'Qmat','Rmat'});

        % Create section for cost function
        AllocationSection = matlab.system.display.Section(...
            'Title','Allocation',...
            'PropertyList',{'Ka'});
        
        SamplingTimeSection = matlab.system.display.Section(...
            'Title','Sampling time',...
            'PropertyList',{'Tsampling'});

        ConstraintsSection = matlab.system.display.Section(...
            'Title','Constraints',...
            'PropertyList',{'hasActuatorBounds',...
            'hasIneqConstraints',...
            'hasSoftConstraints','softConstraintsIndex',...
            'softConstraintsWeight'});

        % Create tab for MPC formulation
        FormulationGroup = matlab.system.display.SectionGroup(...
            'Title','Formulation',...
            'Sections',[MatricesSection, AllocationSection, ...
            SamplingTimeSection, ConstraintsSection]);
        
        % +-----------------------+
        % |     Optional ports    |
        % +-----------------------+
        % Create section for optional output ports
        additionalOutportsSection = matlab.system.display.Section(...
            'Title','Additional Outports', ...
            'PropertyList',{'showSolveTime','showSlackVariable',...
            'showExitflag'});

        % Create tab for general configuration
        portsGroup = matlab.system.display.SectionGroup(...
            'Title','Ports',...
            'Sections', additionalOutportsSection);

        % +-----------------------+
        % |      Create tabs      |
        % +-----------------------+
        group = [FormulationGroup, portsGroup];
    end

    function simMode = getSimulateUsingImpl
        % Return only allowed simulation mode in System block dialog
        simMode = "Interpreted execution";
    end

    function flag = showSimulateUsingImpl
        % Return false if simulation mode hidden in System block dialog
        flag = false;
    end
end

end
