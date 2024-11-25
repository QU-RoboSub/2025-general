classdef BR_Ping < matlab.System ...
        & coder.ExternalDependency ...
        & matlabshared.sensors.simulink.internal.BlockSampleTime

    %
    %#codegen
    %#ok<*EMCA>

    properties

    end

    properties(Access = protected)
        Logo = 'IO Device Builder';
    end

    properties (Nontunable)
        arduinoRxPin = uint8(9);
        arduinoTxPin = uint8(10);
        ledPin = uint8(13);
    end

    properties (Access = private)


    end

    methods
        % Constructor
        function obj = BR_Ping(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods (Access=protected)
        function setupImpl(obj)
            if ~coder.target('MATLAB')
                coder.cinclude('BR_Ping.h');
                coder.ceval('setupFunctionBR_Ping', (obj.arduinoRxPin),1, (obj.arduinoTxPin),1, (obj.ledPin),1);
            end
        end

        function validateInputsImpl(obj,varargin)
            %  Check the input size
            if nargin ~=0



            end
        end

        function Distance = stepImpl(obj)
            Distance = int8(zeros(1,1));

            if isempty(coder.target)
            else
                coder.ceval('stepFunctionBR_Ping',coder.ref(Distance),1);
            end
        end

        function releaseImpl(obj)
            if isempty(coder.target)
            else

            end
        end
    end

    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 0;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function varargout = getInputNamesImpl(obj)

        end

        function varargout = getOutputNamesImpl(obj)
            varargout{1} = 'Distance';
        end

        function flag = isOutputSizeLockedImpl(~,~)
            flag = true;
        end

        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
        end

        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
        end

        function varargout = getOutputSizeImpl(~)
            varargout{1} = [1,1];
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'int8';
        end

        function maskDisplayCmds = getMaskDisplayImpl(obj)
            outport_label = [];
            num = getNumOutputsImpl(obj);
            if num > 0
                outputs = cell(1,num);
                [outputs{1:num}] = getOutputNamesImpl(obj);
                for i = 1:num
                    outport_label = [outport_label 'port_label(''output'',' num2str(i) ',''' outputs{i} ''');' ]; %#ok<AGROW>
                end
            end
            inport_label = [];
            num = getNumInputsImpl(obj);
            if num > 0
                inputs = cell(1,num);
                [inputs{1:num}] = getInputNamesImpl(obj);
                for i = 1:num
                    inport_label = [inport_label 'port_label(''input'',' num2str(i) ',''' inputs{i} ''');' ]; %#ok<AGROW>
                end
            end
            icon = 'BR_Ping';
            maskDisplayCmds = [ ...
                ['color(''white'');',...
                'plot([100,100,100,100]*1,[100,100,100,100]*1);',...
                'plot([100,100,100,100]*0,[100,100,100,100]*0);',...
                'color(''blue'');', ...
                ['text(38, 92, ','''',obj.Logo,'''',',''horizontalAlignment'', ''right'');',newline],...
                'color(''black'');'], ...
                ['text(52,50,' [''' ' icon ''',''horizontalAlignment'',''center'');' newline]]   ...
                inport_label ...
                outport_label
                ];
        end

        function sts = getSampleTimeImpl(obj)
            sts = getSampleTimeImpl@matlabshared.sensors.simulink.internal.BlockSampleTime(obj);
        end
    end

    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end

        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end

    methods (Static)
        function name = getDescriptiveName(~)
            name = 'BR_Ping';
        end

        function tf = isSupportedContext(~)
            tf = true;
        end

        function updateBuildInfo(buildInfo, context)
            coder.extrinsic('codertarget.targethardware.getTargetHardware');
            hCS = coder.const(getActiveConfigSet(bdroot));
            targetInfo = coder.const(codertarget.targethardware.getTargetHardware(hCS));

            % Added this env variable to fetch the comm libraries required only for Arduino target.
            % The env variable is cleared at the end of
            % "GenerateWrapperMakefile.m" file.
            if contains(targetInfo.TargetName,'arduinotarget')
                setenv('Arduino_ML_Codegen_I2C', 'Y');
            end

            buildInfo.addIncludePaths('C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype\Ping');

            buildInfo.addIncludePaths('C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype');
            addSourceFiles(buildInfo,'ping1d.cpp','C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype\Ping');
            addSourceFiles(buildInfo,'BR_Ping.cpp','C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype');

        end
    end
end
