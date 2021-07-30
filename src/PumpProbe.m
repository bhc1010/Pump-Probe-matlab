classdef PumpProbe < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TabGroup                        matlab.ui.container.TabGroup
        PumpProbeTab                    matlab.ui.container.Tab
        AquireDataButton                matlab.ui.control.Button
        EditSettingsButton              matlab.ui.control.Button
        ProbePanel                      matlab.ui.container.Panel
        ProbeSpanEditFieldLabel         matlab.ui.control.Label
        ProbeSpanEditField              matlab.ui.control.NumericEditField
        ProbeAmplitudeEditFieldLabel    matlab.ui.control.Label
        ProbeAmplitudeEditField         matlab.ui.control.NumericEditField
        ProbeEdgeEditFieldLabel         matlab.ui.control.Label
        ProbeEdgeEditField              matlab.ui.control.NumericEditField
        ProbeWidthEditFieldLabel        matlab.ui.control.Label
        ProbeWidthEditField             matlab.ui.control.NumericEditField
        ProbeAmplitudeScale             matlab.ui.control.DropDown
        ProbeEdgeScale                  matlab.ui.control.DropDown
        ProbeWidthScale                 matlab.ui.control.DropDown
        ProbeSpanScale                  matlab.ui.control.DropDown
        PumpPanel                       matlab.ui.container.Panel
        PumpAmplitudeEditFieldLabel     matlab.ui.control.Label
        PumpAmplitudeEditField          matlab.ui.control.NumericEditField
        PumpEdgeEditFieldLabel          matlab.ui.control.Label
        PumpEdgeEditField               matlab.ui.control.NumericEditField
        PumpWidthEditFieldLabel         matlab.ui.control.Label
        PumpWidthEditField              matlab.ui.control.NumericEditField
        PumpWidthScale                  matlab.ui.control.DropDown
        PumpEdgeScale                   matlab.ui.control.DropDown
        PumpAmplitudeScale              matlab.ui.control.DropDown
        LockinPanel                     matlab.ui.container.Panel
        LockinFrequencyEditFieldLabel   matlab.ui.control.Label
        LockinFrequencyEditField        matlab.ui.control.NumericEditField
        SamplesEditFieldLabel           matlab.ui.control.Label
        SamplesEditField                matlab.ui.control.NumericEditField
        LockInFreqScale                 matlab.ui.control.DropDown
        LockinIPEditFieldLabel          matlab.ui.control.Label
        LockinIPEditField               matlab.ui.control.EditField
        PumpProbeLog                    matlab.ui.control.TextArea
        ResetButton                     matlab.ui.control.Button
        SaveDataButton                  matlab.ui.control.Button
        Configurations                  matlab.ui.control.ListBox
        AddConfigurationButton          matlab.ui.control.Button
        RemoveConfigurationButton       matlab.ui.control.Button
        OptionsPanel                    matlab.ui.container.Panel
        AquireMultipleCheckBox          matlab.ui.control.CheckBox
        AutomaticExportCheckBox         matlab.ui.control.CheckBox
        DisablePlottingCheckBox         matlab.ui.control.CheckBox
        EditDefaultPathButton           matlab.ui.control.Button
        Option3CheckBox                 matlab.ui.control.CheckBox
        Option4CheckBox                 matlab.ui.control.CheckBox
        SettingDropDownLabel            matlab.ui.control.Label
        SettingDropDown                 matlab.ui.control.DropDown
        SignalAxes                      matlab.ui.control.UIAxes
        FitDataTab                      matlab.ui.container.Tab
        AxesControlsPanel               matlab.ui.container.Panel
        ReflectionPointKnobLabel        matlab.ui.control.Label
        ReflectionPointKnob             matlab.ui.control.Knob
        VoltageToleranceKnobLabel       matlab.ui.control.Label
        VoltageToleranceKnob            matlab.ui.control.Knob
        AxesLimitsPanel                 matlab.ui.container.Panel
        ReflectionLineSwitchLabel       matlab.ui.control.Label
        ReflectionLineSwitch            matlab.ui.control.Switch
        FlipDataButton                  matlab.ui.control.Button
        ShowFitSwitchLabel              matlab.ui.control.Label
        ShowFitSwitch                   matlab.ui.control.Switch
        Switch2_3Label                  matlab.ui.control.Label
        Switch2_3                       matlab.ui.control.Switch
        Switch2_4Label                  matlab.ui.control.Label
        Switch2_4                       matlab.ui.control.Switch
        ResetViewButton                 matlab.ui.control.Button
        FitDataPanel                    matlab.ui.container.Panel
        FitResults                      matlab.ui.control.TextArea
        FunctionLabel                   matlab.ui.control.Label
        FunctionDropDown                matlab.ui.control.DropDown
        FitButton                       matlab.ui.control.Button
        FitNameEditFieldLabel           matlab.ui.control.Label
        FitNameEditField                matlab.ui.control.EditField
        XdataDropDownLabel              matlab.ui.control.Label
        XdataDropDown                   matlab.ui.control.DropDown
        YdataDropDownLabel              matlab.ui.control.Label
        YdataDropDown                   matlab.ui.control.DropDown
        FitsTable                       matlab.ui.control.Table
        FitFunctionLabel                matlab.ui.control.Label
        FitFunctionEditField            matlab.ui.control.EditField
        RemoveFitButton                 matlab.ui.control.Button
        FitOptionsButton                matlab.ui.control.Button
        AutoFitButton                   matlab.ui.control.StateButton
        ProbeVoltageDataSelectionPanel  matlab.ui.container.Panel
        ListBoxLabel                    matlab.ui.control.Label
        DataSelect                      matlab.ui.control.ListBox
        ImportButton                    matlab.ui.control.Button
        ExportButton                    matlab.ui.control.Button
        GenerateExportFilesCheckBox     matlab.ui.control.CheckBox
        ExportFeedback                  matlab.ui.control.TextArea
        EditPathButton                  matlab.ui.control.Button
        DataAnalysisTypeLabel           matlab.ui.control.Label
        DataAnalysisTypeDropDown        matlab.ui.control.DropDown
        PlotDropDownLabel               matlab.ui.control.Label
        PlotDropDown                    matlab.ui.control.DropDown
        FitAxes                         matlab.ui.control.UIAxes
        ContextMenu                     matlab.ui.container.ContextMenu
        PreferencesMenu                 matlab.ui.container.Menu
    end
   
    properties (Access = private)
        % TCPIP Connections
        LockIn      % Hold connection to Lock-In
        AWG         % Hold connection to AWG
        % Plots
        LockInPlot 
        FitPlot 
        SignalPlot
        % Plot handles
        reflectionLineHandle
        rangeAxes
        rangeLine
        % Data      
        deltaT
        currentData
        fits
        pvdLimits
        fdLimits
        reflectionPoints
        selectedCell
        % Reals
        probeSpan
        probeEdge     
        probeWidth    
        probeAmplitude
        pumpEdge      
        pumpWidth     
        pumpAmplitude
        startWidth
        endWidth
        lockInFreq    
        samples
        currentFit
        % Bools
        connected         % true when connected to system
        changeWaveform    % true when new pulse needs to be generated
        disablePlot       % true when user selects to not plot data from lock-in (greatly increases speed)
        % Strings
        fileName 
        exportPath
        customFunction
        % Fit Options
        FitOptions
    end
    
    % Functions
    methods (Access = private)
        
        % Alana Gudinas
        % 2019
        % Small edits made by Ben Campbell, 2021
        %-------------------------------------------------------------------------------------
        
        % Lock-in setup
        % establish tcpip connection
        function t = lockinSetup(app, IP)
            t = tcpip(IP, 50000);
            
            t.InputBufferSize = 50000;
            
            try
                fopen(t); % open connection
            catch e
                Log(app, e.message);
            end
            %-------- configure lock-in settings --------%
            fprintf(t, 'IE 2'); % set reference mode to external front panel
            pause(0.2)
            fprintf(t, 'IMODE 0'); % current mode off-input voltage only
            pause(0.2)
            fprintf(t, 'VMODE 1'); % A input only
            pause(0.2)
            fprintf(t, 'SEN 19'); % sensitivity 2 mV
            pause(0.2)
            % fprintf(t, 'AS'); % auto-sensitivity
            fprintf(t, 'ACGAIN 5'); % set gain, 6 = 36 dB. dB = 6 * n
            pause(0.2)
            % fprintf(t, 'AUTOMATIC 0'); % set AC gain to automatic control
            fprintf(t, 'AQN'); % auto-phase
            pause(0.2)
            % fprintf(t, 'TC 10'); % set filter time constant control to 20 ms
            % fprintf(t, 'FLOAT 1'); % 'float input connector shell using 1kOhm to ground' need to read more
            % pause(0.1)
            fprintf(t, 'LF 0'); % 'turn off line frequency rejection filter
            pause(0.1)
            %-------------------------------------------%
            % program should wait a period of four time-constants after input signal
            % is changed before recording a new value (from manual) 
            pause(0.1)
        end
        
        % AWG setup
        % establish visa USB connection
        function v = awgSetup(app)     
            rsName = 'USB0::0x0957::0x5707::MY53805152::0::INSTR'; % from NI MAX
            
            v = instrfind('Type', 'visa-usb', 'RsrcName', rsName , 'Tag', '');
            % Create the VISA-TCPIP object if it does not exist
            % otherwise use the object that was found.
            if isempty(v)
                v = visa('ni', rsName);
            else
                fclose(v);
                v = v(1);
            end
            
            try
                fopen(v);
            catch e
               Log(app, e.message); 
            end
            pause(1);
        end
        
        % Resets AWG and LockIn
        function reset(app, v, t)
            fprintf(v, '*RST'); % reset AWG
            fprintf(t, '*CLS'); % clear AWG
            app.changeWaveform = true;
            pause(8);
        end
        
        % Create pulse vector from input
        function [prTemp, ppTemp] = generatePulseVector(app, probeSpan, probeEdge, probeWidth, pumpEdge, pumpWidth)
            prTemp = createPulse(app, probeWidth, probeEdge, probeSpan); %creates the probe Arb
            ppTemp = createPulse(app, pumpWidth, pumpEdge, probeSpan);
            pause(0.1);
        end
        
        % Send waveforms to AWG
        function waveform2AWG(app, ppTemp, pumpAmplitude, prTemp, probeAmplitude)
            sendArbCh(app, ppTemp, pumpAmplitude, 1e9, 'wpp', 1) % channel 1
            sendArbCh(app, prTemp, probeAmplitude, 1e9, 'wpr', 2) % channel 2
            pause(2);
        end
        
        % Set Lock-In Modulation
        % lockInFreq = desired approximate lock in frequency in Hz
        function modulateLockIn(app, v, lockInFreq)
            chNum = 2; % modulate on channel 2
            amplitudeModulation(app, lockInFreq, chNum, v);
            fprintf(v,'SOURce1:FUNC:ARB:SYNC');%sync both channels
            fprintf(v, '*WAI');
            fprintf(v,'SOURce1:COMBine:FEED CH2'); %combine CH1 and CH2 in CH1 output
            fprintf(v, '*WAI');
            pause(4);
        end
        
        function setLIPlot(app)
            cla(app.SignalAxes, 'reset'); % reset Signal plot
            if ~app.DisablePlottingCheckBox.Value
                app.SignalPlot = plot(app.SignalAxes, deltat(1), data(1),'.-','MarkerSize',13,'LineWidth',0.5,'XDataSource','deltat','YDataSource','data','Color',[0.4 0.1 1]);
                grid(app.SignalAxes, 'on');
            end
        end

        function x = parseLI(app)
            fprintf(t, 'X.'); % read value from Lock-in
            val = fscanf(t);
            for k = ['c', '#', '"']
                test = strsplit(val, k);
                n = max(size(test));
                val = test{n};
            end
            x = sscanf(val,'%f'); % correctly converts string to fp number
        end

        function [data, deltat] = modulatePhase(app, v, t, nbs, probeSpan)
            prange = 180;
            deltaRange = linspace(-prange,prange,nbs); % define the angle through which to sweep
            inDwell=2; % inital dwell to clear lockin buffer
            
            % go to first phase to allow settling down of buffer
            myString = sprintf('SOURce1:PHASe:ARB %g',deltaRange(1));
            fprintf(v, myString);
            pause(inDwell);
            
            fprintf(t, 'AQN'); % auto-phase
            pause(0.2)

            % Data acquisition
            setLIPlot(app);
            data=zeros(1,nbs);
            deltat=zeros(1,nbs);
            for i=1:nbs
                deltat(i) = deltaRange(i)/360*probeSpan*1e9; %time label for Plot
                c = sprintf('SOURce1:PHASe:ARB %g',deltaRange(i));
                fprintf(v, c);
                fprintf(v,'*WAI');
                x = parseLI(app);
                try 
                    data(i) = x;
                catch e 
                    Log(app, e.message);
                    data(i) = [];
                    deltat(i) = [];
                end
                if ~app.DisablePlottingCheckBox.Value
                    set(app.SignalPlot, "XData", deltat(1:i), "YData", data(1:i));
                end
                pause(0.00001);
            end
            if ~app.DisablePlottingCheckBox.Value
                set(app.SignalAxes, "XLim", [deltat(1) deltat(end)]);
            end
            hold(app.SignalAxes, 'off');
        end

        function [data, deltaW] = modulateWidth(app, v, t, nbs, probeSpan)
            setLIPlot(app);
            data=zeros(1,nbs);
            deltaW = linspace(app.startWidth, app.endWidth, nbs); % define the angle through which to sweep

            % Data acquisition
            for i=1:nbs                
                % Send new waveform
                prTemp = createPulse(app, deltaW(i), app.probeEdge, app.probeSpan + deltaW(end)); %creates the probe Arb
                sendArbCh(app, prTemp, probeAmplitude, 1e9, 'wpr', 2) % channel 2
                pause(1);
                c = sprintf('SOURce2:PHASe:ARB %g', deltaW(i));
                fprintf(v, c);
                fprintf(v,'*WAI');
                x = parseLI(app);
                try 
                    data(i) = x;
                catch e 
                    Log(app, e.message);
                    data(i) = [];
                    deltaW(i) = [];
                end
                if ~app.DisablePlottingCheckBox.Value
                    set(app.SignalPlot, "XData", deltaW(1:i), "YData", data(1:i));
                end
                pause(0.00001);
            end
            if ~app.DisablePlottingCheckBox.Value
                set(app.SignalAxes, "XLim", [deltaW(1) deltaW(end)]);
            end
            hold(app.SignalAxes, 'off');
        end

        % Run the pump-probe algorithm
        % function [data, deltat] = runPumpProbe(app, v, t, nbs, probeSpan, modFn)
            
        %     %---------------------------------------------
        %     % Data acquisition
        %     %---------------------------------------------
        %     setLIPlot(app);
        %     data=zeros(1,nbs); % initialize values
        %     deltat=zeros(1,nbs);
        %     for i=1:nbs
        %         deltat(i) = deltaRange(i)/360*probeSpan*1e9; %time label for Plot
        %         c = sprintf('SOURce1:PHASe:ARB %g',deltaRange(i));
        %         fprintf(v, c);
        %         fprintf(v,'*WAI');
        %         x = parseLI(app);
        %         try 
        %             data(i) = x;
        %         catch e 
        %             Log(app, e.message);
        %             deltat(i) = [];
        %         end
        %         if ~app.DisablePlottingCheckBox.Value
        %             set(app.SignalPlot, "XData", deltat(1:i), "YData", data(1:i));
        %         end
        %         pause(0.00001);
        %     end
        %     if ~app.DisablePlottingCheckBox.Value
        %         set(app.SignalAxes, "XLim", [deltat(1) deltat(end)]);
        %     end
        %     hold(app.SignalAxes, 'off');
        %     % beep
        % end
        %------------------------------------------------------------------------------------------------
        
        % by FDN 
        %   v0.1    Nov 2018    creates pulses for Pump Probe AWG
        function [pulse, risingEdge, fallingEdge] = createPulse(app, width, risetime, timespread)
            %timespread in seconds
            minRiseTime = 4e-9; %4ns is the smallest rise time
            minPulseWidth = 4e-9; %5ns is the minimal pulse width
            samplerate=1e9;%hardcoded sample rate, going for max value 1GSa/s at the moment
            timeIncrement = 1/samplerate; %time step size
            minArbLength = 32; %minimal length of ARB waveform
            risingEdge = linspace(0,1,max(round(minRiseTime/timeIncrement),round(risetime/timeIncrement))); %linear rise, 
            fallingEdge = fliplr(risingEdge); %create falling edge pulse, just mirrow of rising
            if width>minPulseWidth
                pulse = [risingEdge ones(1,max(round(minPulseWidth/timeIncrement),round(width/timeIncrement)-length(risingEdge))) fallingEdge];
            else
                Log(app, 'Pulse generated with 1ns width')
                pulse = [risingEdge fallingEdge];
            end
            if length(pulse)<max(minArbLength,timespread*samplerate)
                pulse = [pulse zeros(1,max(minArbLength,round(timespread*samplerate))-length(pulse))]; %pad waveform with zeros
                %pulse = [zeros(1,minArbLength-length(pulse)) pulse];
            end
        end

        function [pulse, risingEdge, fallingEdge] = createInterlacedPulse(app, width, risetime, timespread, interlace)
            %timespread in seconds
            minRiseTime = 4e-9; %4ns is the smallest rise time
            minPulseWidth = 4e-9; %5ns is the minimal pulse width
            samplerate=1e9;%hardcoded sample rate, going for max value 1GSa/s at the moment
            timeIncrement = 1/samplerate; %time step size
            minArbLength = 32; %minimal length of ARB waveform
            risingEdge = linspace(0,1,max(round(minRiseTime/timeIncrement),round(risetime/timeIncrement))); %linear rise, 
            fallingEdge = fliplr(risingEdge); %create falling edge pulse, just mirrow of rising
            if width>minPulseWidth
                arbA = [risingEdge ones(1,max(round(minPulseWidth/timeIncrement),round(width/timeIncrement)-length(risingEdge))) fallingEdge];
            else
                Log(app, 'Pulse generated with 1ns width')
                arbA = [risingEdge fallingEdge];
            end
            if length(arbA)<max(minArbLength,timespread*samplerate)
                pulseA = [arbA zeros(1,max(minArbLength,round(timespread*samplerate))-length(arbA))]; %pad waveform with zeros-
            end
            arbB = interlace * arbA;
            if length(arbB)<max(minArbLength,timespread*samplerate)
                pulseB = [arbB zeros(1,max(minArbLength,round(timespread*samplerate))-length(arbB))]; %pad waveform with zeros-
            end
            pulse = [pulseA pulseB];
        end

        % Alana Gudinas
        % July 2019 
        %
        % This is a script to program amplitude modulation (sq wave) on a specified channel
        % for the Keysight 33600A arbitrary waveform generator. 
        % 
        % "chNum" is an integer input and must be either 1 or 2 to indicate which
        % channel will have amplitude modulation.
        % "freq" specifies the modulaton frequency.
        %
        % This script assumes that the output will be synced on the input channel,
        
        function amplitudeModulation(~, freq, chNum, t)
        
            sName = sprintf('SOURCE%d:',chNum);
                
            c = strcat(sName,'AM:DEPT 100');
            fprintf(t, c); %set Am deviation to 100%
            fprintf(t, '*WAI'); 
            
            c = strcat(sName,'AM:DSSC OFF');
            fprintf(t, c);
            fprintf(t, '*WAI');  % DSSC OFF: amplitude is zero in second cycle
        
            c = strcat(sName,'AM:SOURCE INT');
            fprintf(t, c); %define internal modulation
            fprintf(t, '*WAI'); 
            
            c = strcat(sName,'AM:INT:FUNC SQU');
            fprintf(t, c); %define internal modulation function
            fprintf(t, '*WAI'); 
            
            myString = sprintf('AM:INT:FREQ %g',freq); %lock-in period
            c = strcat(sName,myString);
            fprintf(t, c);
            fprintf(t, '*WAI');
            
            c = strcat(sName,'AM:STATE ON');
            fprintf(t, c); %active amplitude modulation
            fprintf(t, '*WAI');
        
            c = sprintf('OUTP:SYNC:SOURCE CH%d',chNum);
            fprintf(t, c);%sync to chNum AM
            fprintf(t, '*WAI');
            
            fprintf(t,'SOURce1:PHASe:SYNChronize'); % source number is arbitrary
            
        end
        
        % This function connects to a 33600A waveform generator and sends it an
        % arbitrary waveform from Matlab via USB. The input arguments are as
        % follows:
        % arb --> a vector of the waveform points that will be sent to a 33600A
        % waveform generator
        % sRate --> sample rate of the arb waveform
        % name --> The same of the arb waveform as a string
        % channel --> channel number, 1 or 2
        % Note: this function requires the instrument control toolbox
        function sendArbCh(app,arb,amp,sRate,name,channel)
            
            % Establish connection via VISA USB
            v = instrfind('Type', 'visa-usb', 'RsrcName', 'USB0::0x0957::0x5707::MY53805152::0::INSTR', 'Tag', '');
            % Create the VISA-TCPIP object if it does not exist
            % otherwise use the object that was found.
            if isempty(v)
                v = visa('ni', 'USB0::0x0957::0x5707::MY53805152::0::INSTR');
            else
                fclose(v);
                v = v(1);
            end
            
            % Set size of receiving buffer, if needed. 
            set(v, 'InputBufferSize', 30000); 
            v.Timeout = 15; %set IO time out
            
            % calculate output buffer size
            buffer = length(arb)*8;
            set(v,'OutputBufferSize',(buffer+128));
            
            fopen(v);
            
            if channel == 1
                sName = 'SOURce1:';
                oN = 'OUTPUT1 ON';
            elseif channel == 2
                sName = 'SOURce2:';
                oN = 'OUTPUT2 ON';
            end
                
            % Query Identity string and report
            fprintf (v, '*IDN?');
            idn = fscanf(v);
            fprintf (idn);
            fprintf ('\n\n');
            
            % make sure waveform data is in column vector
            if ~isrow(arb)
                arb = arb';
            end
            
            % set the waveform data to single precision
            arb = single(arb);
            
            % scale data between 1 and -1
            mx = max(abs(arb));
            arb = arb/mx;
            
            % send waveform to 33600
            c = strcat(sName,'DATA:VOLatile:CLEar');
            fprintf(v, c); %Clear volatile memory
            fprintf(v, 'FORM:BORD SWAP');  %configure the box to correctly accept the binary arb points
            
            arbBytes=num2str(length(arb) * 4); %# of bytes
            
            header= [sName 'DATA:ARBitrary ' name ', #' num2str(length(arbBytes)) arbBytes]; %create header
            binblockBytes = typecast(arb, 'uint8');  %convert datapoints to binary before sending
            fwrite(v, [header binblockBytes], 'uint8'); %combine header and datapoints then send to instrument
            fprintf(v, '*WAI');   %Make sure no other commands are exectued until arb is done downloading

            % Set desired configuration for channel 1
            command = ['FUNCtion:ARBitrary ' name];
            command = strcat(sName,command);
            fprintf(v,command); % set current arb waveform to defined arb testrise
            command = ['MMEM:STOR:DATA1 "INT:\' name '.arb"'];
            fprintf(v,command);
            command = ['FUNCtion:ARB:SRATe ' num2str(sRate)]; %create sample rate command
            command = strcat(sName,command);
            fprintf(v,command);%set sample rate
            c = strcat(sName,'FUNCtion ARB');
            fprintf(v,c); % turn on arb function
            command = ['VOLT ' num2str(amp)]; %create amplitude command
            command = strcat(sName,command);
            fprintf(v,command); %send amplitude command
            c = strcat(sName,'VOLT:OFFSET 0');
            fprintf(v,c); % set offset to 0 V
            fprintf(v,oN); %Enable Output for channel 1
            
            % Read Error
            fprintf(v, 'SYST:ERR?');
            errorstr = fscanf(v);
            % error checking
            if strncmp (errorstr, '+0,"No error"',13)
               errorcheck = 'Arbitrary waveform generated without any error';
               Log(app, errorcheck);
            else
               errorcheck = ['Error reported: ', errorstr];
               Log(app, errorcheck);
            end
        end 
        
        % Ben Campbell 
        % v0.1 - 2021
        function Tau = getTimeConstant(app, data)
            [ref,~] = closest(app, data(2,:), app.ReflectionPointKnob.Value);
            timeDependent = data(1, ref+1:end);
            len = length(timeDependent);
            if ref - len > 0
                background = data(1, ref-len+1:ref);
                tau = timeDependent - flip(background);
                deltat = data(2, ref+1:end);
            else 
                background = data(1, 1:ref);
                tau = timeDependent(1:ref) - flip(background);
                deltat = data(2, ref:2*ref-1);
            end
            % if min(tau) < 0
            %     tau = tau + abs(min(tau)) + 1e-6;
            % end
            I = tau > app.VoltageToleranceKnob.Value;
            tau(I) = [];
            deltat(I) = [];
            Tau = [tau; deltat];
        end

        % GUI functions
        % Ben Campbell

        function redrawFitAxes(app)
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')
                data = app.DataSelect.Value{1};
                app.FitPlot = plot(app.FitAxes, data(2,:), data(1,:), '.','MarkerSize',6,'LineWidth',1.5,'Color',[0.4 0.1 1]);
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                tau = app.DataSelect.Value{2};
                app.FitPlot = plot(app.FitAxes, tau(2,:), tau(1,:), '.','MarkerSize',6,'LineWidth',1.5,'Color',[0.4 0.1 1]);
            end
        end
        
        function plotFit(app)
            % Redraw Fit Axes (removes previous fitting line)
            redrawFitAxes(app);
            % Draw current fit onto Fit Axes
            fitobject = app.fits{app.currentFit}{1};
            x = app.FitPlot.XData;
            lims = app.FitsTable.Data{app.currentFit, 6};
            lims = strsplit(lims, ',');
            ll = str2double(lims{1}(2:end));
            rl = str2double(lims{2}(1:end-1));
            [i1, ~] = closest(app, x, ll);
            [i2, ~] = closest(app, x, rl);
            x = x(i1:i2);
            y = feval(fitobject, x);
            hold(app.FitAxes, 'on');
            plot(app.FitAxes, x, y);
            hold(app.FitAxes, 'off');
            set(app.FitAxes, 'XLim', [ll rl]);
            % Display fitting info
            fitlog = evalc('fitobject');
            app.FitResults.Value = fitlog(15:end);
        end

        function drawAxesLimits(app)
            if ~isempty(app.rangeLine)
                delete(app.rangeLine)
            end
            % Create the 1D axes
            app.rangeAxes = axes( ...
            app.AxesLimitsPanel,'Color','none','YColor','none','XLim',[-50,50],'YTick',[], ...
            'XTick',-50:10:50,'TickDir','both','TickLength',[0.03,0.035],'XMinorTick', ...
            'on','Units','pixels','Position',[8,32,298,0]);
            % Disable the interactivity & toolbar visibility
            disableDefaultInteractivity(app.rangeAxes);
            app.rangeAxes.Toolbar.Visible = 'off';
            % Add the line ROI
            app.rangeLine = images.roi.Line(app.rangeAxes,'Position',[-50,0;50,0]);
            % Add a listener that will trigger a callback function titled "lMoving" when user
            % moves the ROI endpoints or the line ROI as a whole
            addlistener(app.rangeLine,'MovingROI',@(varargin)lMoving(app,app.rangeLine));
        end      
        
        % Draws Reflection Line : x is the value of an element of data
        function drawReflectionLine(app, x)
            data = app.DataSelect.Value{1};
            [idx, ~] = closest(app, data(2,:), x);
            if isempty(app.reflectionLineHandle)
                app.reflectionLineHandle = xline(app.FitAxes, data(2, idx), '--r');
            else 
                app.reflectionLineHandle.Value = data(2,idx);
            end
        end
        
        function drawLockIn(app)
            data = app.DataSelect.Value{1};
            app.LockInPlot = plot(app.FitAxes, data(2,:), data(1,:),'.','MarkerSize',6,'LineWidth',0.5,'Color',[0.4 0.1 1]);
        end      
                
        function result = maybeChangeWaveform(app)
            prEdge = app.ProbeEdgeEditField.Value * app.ProbeEdgeScale.Value; 
            prWidth = app.ProbeWidthEditField.Value * app.ProbeWidthScale.Value;        
            prAmp = app.ProbeAmplitudeEditField.Value * app.ProbeAmplitudeScale.Value;
            ppEdge = app.PumpEdgeEditField.Value * app.PumpEdgeScale.Value;            
            ppWidth = app.PumpWidthEditField.Value * app.PumpWidthScale.Value;
            ppAmp = app.PumpAmplitudeEditField.Value * app.PumpAmplitudeScale.Value;
            if app.probeEdge ~= prEdge || app.probeWidth ~= prWidth || app.probeAmplitude ~= prAmp || app.pumpEdge ~= ppEdge || app.pumpWidth ~= ppWidth || app.pumpAmplitude ~= ppAmp || app.changeWaveform == true
                result = true;
                app.changeWaveform = false;
                app.probeEdge = prEdge;
                app.probeWidth = prWidth;
                app.probeAmplitude = prAmp;
                app.pumpEdge = ppEdge;
                app.pumpWidth = ppWidth;
                app.pumpAmplitude = ppAmp;
            else 
                result = false;
            end
        end

        function name = getConfigName(app)
            ppA = app.PumpAmplitudeEditField.Value;
            ppE = app.PumpEdgeEditField.Value;
            ppW = app.PumpWidthEditField.Value;
            prA = app.ProbeAmplitudeEditField.Value;
            prE = app.ProbeEdgeEditField.Value;
            prW = app.ProbeWidthEditField.Value;
            prS = app.ProbeSpanEditField.Value;
            name = convertStringsToChars('pp-A' + string(ppA) + '-E' + string(ppE) + '-W' + string(ppW) + '-pr-A' + string(prA) + '-E' + string(prE) + '-W' + string(prW) + '-S' + string(prS));
        end

        function saveLockInData(app)
            if ~app.AutomaticExportCheckBox.Value
                [file, path] = uiputfile('*.csv');
                figure(app.UIFigure);
                if file
                    writematrix([app.currentData; app.deltaT], fullfile(path, file)); % save data to csv file.
                end
            else
                if ~app.AquireMultipleCheckBox.Value
                    myPath = fullfile(app.exportPath, app.fileName);
                    if ~isfolder(path)
                        mkdir(path)
                    end
                    myPath = fullfile(myPath, append(app.fileName, '.csv'))
                    writematrix([app.currentData; app.deltaT], myPath);
                    Log(app, append("Data saved to ", myPath));
                else
                    % path = uigetdir();
                    path = fullfile(app.exportPath, app.fileName);
                    figure(app.UIFigure);
                    % if ~isfolder(path)
                    %     mkdir(path)
                    % end
                    for i = 1:length(app.currentData)
                        p = fullfile(path, app.Configurations.Items{i});
                        try
                            mkdir(p);
                        catch e 
                            msgbox(e.message);
                        end
                        writematrix([app.currentData{i}; app.deltaT{i}], fullfile(p, append(app.Configurations.Items{i}, '.csv')));
                    end
                end
            end
        end

        function Log(app, message)
            if isempty(app.PumpProbeLog.Value{1})
               line = '';
            else
                line = sprintf('%s\n', string(app.PumpProbeLog.Value));
            end
            set(app.PumpProbeLog, 'Value', append(line, message));
        end
        
        function [idx, val] = closest(~, testArr, val)
            tmp = abs(testArr - val);
            [~, idx] = min(tmp);
            val = testArr(idx);
        end

        % Function that responds to the movement of the sliders
        function lMoving(app, source)
            % Always keep the position rounded to 2 decimal places
            source.Position(1,1) = round(source.Position(1,1),2);
            source.Position(2,1) = round(source.Position(2,1),2);
            % Add any other action here triggered by the movement of the sliders
            set(app.FitAxes, 'XLim', [source.Position(1,1) source.Position(2,1)]);
            idx = app.DataSelect.Value{3};
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')
                app.pvdLimits(idx, 1) = source.Position(1,1);
                app.pvdLimits(idx, 2) = source.Position(2,1);
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                app.fdLimits(idx, 1) = source.Position(1,1);
                app.fdLimits(idx, 2) = source.Position(2,1);
            end
        end

        function fo = defaultFitOptions(~, len)
            fo = fitoptions('exp1');
            fo.StartPoint = ones(1,len);
            fo.Lower = -Inf * ones(1, len);
            fo.Upper = Inf * ones(1, len);
        end

    end
 
    methods (Access = public)
        function updateFitOptions(app, fo)
            app.FitOptions = fo;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            disableDefaultInteractivity(app.FitAxes);
            disableDefaultInteractivity(app.SignalAxes);
            app.connected = false;
            app.changeWaveform = true;
            app.VoltageToleranceKnob.Value = 1;
            app.probeSpan      = 0;
            app.probeEdge      = 0;
            app.probeWidth     = 0;
            app.probeAmplitude = 0;
            app.pumpEdge       = 0;
            app.pumpWidth      = 0;
            app.pumpAmplitude  = 0;
            app.startWidth     = 10e-9;
            app.endWidth       = 50e-9;
            app.lockInFreq     = 0;
            app.samples        = 0;     

            app.fits = {};
            FunctionDropDownValueChanged(app);
            app.currentFit = 0;
            app.customFunction = 'a + b*exp(-x/T)';

            % Create the 1D axes
            drawAxesLimits(app);

            app.exportPath = pwd;
        end

        % TODO: ADD AUTOSAVE TO AUTOMATIC EXPORT OPTION
        % Button pushed function: AquireDataButton
        function AquireDataButtonPushed(app, ~)
            app.AquireDataButton.Enable = false;
            app.ResetButton.Enable = false;
            app.SaveDataButton.Enable = false;
            app.PumpProbeLog.Value = '';
            % If not connected to Lock-In or AWG, then connect to both
            if ~app.connected
                Log(app, 'Connecting to Lock-In via IP');
                try
                    app.LockIn = lockinSetup(app, app.LockinIPEditField.Value);
                catch e
                    Log(app, e.message);
                    app.AquireDataButton.Enable = true;
                    return;
                end
                Log(app, 'Connecting to AWG via USB');
                try 
                    app.AWG = awgSetup(app);
                catch e
                    Log(app, e.message)
                    app.AquireDataButton.Enable = true;
                    return;
                end
                Log(app, 'Resetting Lock-In / AWG')
                reset(app, app.AWG, app.LockIn);
                app.connected = true;
            end
            if ~app.AquireMultipleCheckBox.Value
                app.probeSpan = app.ProbeSpanEditField.Value * app.ProbeSpanScale.Value;
                app.samples = app.SamplesEditField.Value;
                if maybeChangeWaveform(app)
                    Log(app, 'Generating Pulse');
                    [pptemp, prtemp] = generatePulseVector(app, app.probeSpan, app.probeEdge, app.probeWidth, app.pumpEdge, app.pumpWidth);
                    Log(app, 'Sending waveform to AWG');
                    waveform2AWG(app, pptemp, app.pumpAmplitude, prtemp, app.probeAmplitude);
                    modulateLockIn(app, app.AWG, app.lockInFreq);
                end
                if app.lockInFreq ~= app.LockinFrequencyEditField.Value * app.LockInFreqScale.Value
                    Log(app, 'Setting Lock-in to modulation frequency');
                    app.lockInFreq = app.LockinFrequencyEditField.Value * app.LockInFreqScale.Value;
                    modulateLockIn(app, app.AWG, app.lockInFreq);
                end
                Log(app, 'Running Pump-Probe')
                % tic
                [app.currentData, app.deltaT] = runPumpProbe(app, app.AWG, app.LockIn, app.samples, app.probeSpan);
                % toc
                app.fileName = getConfigName(app);
                app.DataSelect.Items{end+1} = app.fileName;
                data = [app.currentData; app.deltaT];
                tau = getTimeConstant(app, data);
                app.pvdLimits(end+1, :) = [data(2,1) data(2, end)];
                app.fdLimits(end+1, :) = [0 tau(2, end)];
                app.DataSelect.ItemsData{end+1} = {data; tau; length(app.DataSelect.Items)};
                if app.AutomaticExportCheckBox 
                    saveLockInData(app);
                end
            else
                Log(app, 'Collecting Multiple Data');
                app.currentData = {};
                app.deltaT = {};
                app.lockInFreq = app.LockinFrequencyEditField.Value * app.LockInFreqScale.Value;
                app.samples = app.SamplesEditField.Value;
                for i = 1:length(app.Configurations.ItemsData)
                    config = app.Configurations.ItemsData{i};
                    ppA = config{1};
                    ppE = config{2};
                    ppW = config{3};
                    prA = config{4};
                    prE = config{5};
                    prW = config{6};
                    prS = config{7};
                    Log(app, 'Generating Pulse ' + string(i));
                    [pptemp, prtemp] = generatePulseVector(app, prS, prE, prW, ppE, ppW);
                    Log(app, 'Sending waveform ' + string(i) + ' to AWG');
                    waveform2AWG(app, pptemp, ppA, prtemp, prA);
                    Log(app, 'Setting Lock-in to modulation frequency');
                    modulateLockIn(app, app.AWG, app.lockInFreq);
                    Log(app, 'Running Pump-Probe');
                    [app.currentData{i}, app.deltaT{i}] = runPumpProbe(app, app.AWG, app.LockIn, app.samples, prS);
                    app.DataSelect.Items{end+1} = app.Configurations.Items{i};
                    data = [app.currentData{i}; app.deltaT{i}];
                    tau = getTimeConstant(app, data);
                    app.pvdLimits(end+1, :) = [data(2,1) data(2, end)];
                    app.fdLimits(end+1, :) = [0 tau(2, end)];
                    app.DataSelect.ItemsData{end+1} = {data; tau; length(app.DataSelect.Items)};
                end
            end
            app.AquireDataButton.Enable = true;
            if app.connected
                app.ResetButton.Enable = true;
            end
            app.SaveDataButton.Enable = true;
        end

        function EditSettingsButtonPushed(app, ~)
            setting = app.SettingDropDown.Value;
            dlgtitle = 'Experiment Settings';
            dims = [1 35];
            switch setting 
                case 'Sweep Phase'
                    prompt = {'Start phase:', 'End phase:'};
                    defaultIn = {'-180', '180'};
                    newSetting = inputdlg(prompt, dlgtitle, dims, defaultIn);
                case 'Sweep DC Offset (IV)'
                    prompt = {'Initial DC offset (mV)', 'Final DC offset (mV)'};
                    defaultIn = {'600', '700'};
                    newSetting = inputdlg(prompt, dlgtitle, dims, defaultIn);
            end
        end

        % Value changed function: DataSelect
        function DataSelectValueChanged(app, ~)
            data = app.DataSelect.Value{1};
            tau = app.DataSelect.Value{2};
            idx = app.DataSelect.Value{3};
            % Plotting
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')
                app.FitPlot = plot(app.FitAxes, data(2,:), data(1,:), '.','MarkerSize',6,'LineWidth',0.5,'Color',[0.4 0.1 1]);
                set(app.rangeLine, 'Position', [-50, 0; 50, 0]);
                set(app.FitAxes, 'XLim', [-50 50]);
                set(app.rangeAxes, 'XLim', [-50,50], 'XTick', -50:10:50);
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                app.FitPlot = plot(app.FitAxes, tau(2,:), tau(1,:), '.','MarkerSize',6,'LineWidth',0.5,'Color',[0.4 0.1 1]);
                set(app.rangeLine, 'Position', [app.fdLimits(idx, 1), 0; app.fdLimits(idx, 2), 0]);
                set(app.FitAxes, 'XLim', [app.fdLimits(idx,1) app.fdLimits(idx, 2)]);
                set(app.rangeAxes, 'XLim', [0,50], 'XTick', 0:10:50);
            end
            % Reflection Line
            app.ReflectionPointKnob.Value = app.reflectionPoints(idx);
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data') && app.ReflectionLineSwitch.Value
                delete(app.reflectionLineHandle);
                app.reflectionLineHandle = [];
                drawReflectionLine(app, app.ReflectionPointKnob.Value);
            end
            % Enable controls (need only do once, but I don't know how to do that...)
            app.AxesControlsPanel.Enable = 'on';
            app.FitDataPanel.Enable = 'on'; 
        end

        % Button pushed function: ImportButton
        function ImportButtonPushed(app, ~)
            [file, path] = uigetfile({'*.csv';'*.txt'},'Select Data');
            figure(app.UIFigure);
            if file ~= 0
                app.DataSelect.Items{end+1} = file;
                fileData = readmatrix(fullfile(path, file));
                tau = getTimeConstant(app, fileData);
                idx = length(app.DataSelect.Items);
                data = {fileData; tau; idx; path};
                app.DataSelect.ItemsData{end+1} = data;
                app.pvdLimits(idx, :) = [fileData(2,1) fileData(2, end)];
                app.fdLimits(idx, :) = [0 tau(2, end)];
                app.reflectionPoints(idx) = 0;
                app.ExportButton.Enable = 'on';
            end
        end    

        % Button pushed function: ExportButton
        function ExportButtonPushed(app, ~)

            if ~app.GenerateExportFilesCheckBox.Value
                [file, path] = uiputfile('*.csv');
                figure(app.UIFigure);
                path = fullfile(path, 'fits');
                fname = file(1:end-4);
            else
                idx = app.DataSelect.Value{3};
                fname = app.DataSelect.Items{idx};
                fname = fname(1:end-4);
                idx = app.DataSelect.Value{3};
                path = fullfile(app.DataSelect.Value{4}, fullfile('fits', app.FitsTable.Data{app.currentFit, 1}));
            end

            if fname ~= 0
                if ~isfolder(path)
                    try
                        mkdir(path);
                    catch e
                        msgbox(e.message);
                        return
                    end
                end
                % Write Log to file
                writecell(app.FitResults.Value, fullfile(path, append(fname, '.txt')));

                % Write fit data to file
                if ~isempty(app.fits)
                    x = app.FitPlot.XData;
                    y = app.FitPlot.YData;
                    i1 = app.fits{app.currentFit}{2};
                    i2 = app.fits{app.currentFit}{3};
                    x = x(i1:i2);
                    y = y(i1:i2);
                    writematrix([x;y], fullfile(path, append(fname, '.csv')));
                end

                % Write figure to file
                h = figure('visible', 'off');
                axNew = axes; 
                % Copy all objects from FitAxes to new axis
                copyobj(app.FitAxes.Children, axNew);
                % Save all parameters of the FitAxes
                uiAxParams = get(app.FitAxes);
                uiAxParamNames = fieldnames(uiAxParams); 
                % Get list of editable params in new axis
                editableParams = fieldnames(set(axNew)); 
                % Remove the FitAxes params that aren't editable in the new axes (add others you don't want)
                badFields = uiAxParamNames(~ismember(uiAxParamNames, editableParams)); 
                badFields = [badFields; 'Parent'; 'Children'; 'XAxis'; 'YAxis'; 'ZAxis';'Position';'OuterPosition']; 
                uiAxGoodParams = rmfield(uiAxParams,badFields); 
                % set editable params on new axes
                set(axNew, uiAxGoodParams);
                set(axNew, 'Position', [55, 0, 500, 450]);
                title(axNew, 'Exponential fit to time-dependent signal');
                xlabel(axNew, 'Time Delay (ns)');
                ylabel(axNew, 'Time-dependent Signal (V)');
                saveas(h, fullfile(path, append(fname, '.fig')));
                delete(h);

                app.ExportFeedback.Value = sprintf('%s','Exported sucessfully to: ', path);
            else
                app.ExportFeedback.Value = sprintf('%s', 'Export Failed. Attempted to export to ', path);
            end
        end

        % Value changed function: PlotDropDown
        function PlotDropDownValueChanged(app, event)
            p = app.PlotDropDown.Value;
            idx = app.DataSelect.Value{3};
            if strcmp(p, 'Probe Voltage Data')
                data = app.DataSelect.Value{1};
                set(app.FitPlot, 'XData', data(2,:), 'YData', data(1,:));
                set(app.FitAxes, 'XLim', [-50 50]);
                ylabel(app.FitAxes, 'Probe Voltage (mV)');
                set(app.rangeLine, 'Position', [-50, 0;50, 0]);
                set(app.rangeAxes, 'XLim', [-50,50], 'XTick', -50:10:50);
                if app.ReflectionLineSwitch.Value
                    drawReflectionLine(app, app.ReflectionPointKnob.Value)
                end
            elseif strcmp(p, 'Calculated Data')
                data = app.DataSelect.Value{2};
                set(app.FitPlot, 'XData', data(2,:), 'YData', data(1,:));
                set(app.FitAxes, 'XLim', [0 50]);
                ylabel(app.FitAxes, 'Calculated variable (mV)');
                set(app.rangeLine, 'Position', [0, 0;50, 0]);
                set(app.rangeAxes, 'XLim', [0,50], 'XTick', 0:10:50);
                delete(app.reflectionLineHandle);
                app.reflectionLineHandle = [];
            end
            VoltageToleranceKnobValueChanged(app, event);
        end

        % Value changed function: FunctionDropDown
        function FunctionDropDownValueChanged(app, ~)
            f = app.FunctionDropDown.Value;
            switch f 
                case 'Exponential'
                    app.FitFunctionEditField.Value = 'a*exp(b*x)';
                case 'Polynomial'
                    app.FitFunctionEditField.Value = 'a*x^2 + b*x + c';
                case 'Linear'
                    app.FitFunctionEditField.Value = 'm*x + b';
                case 'Custom Function' 
                    app.FitFunctionEditField.Value = app.customFunction;
            end
            coeffs = coeffnames(fittype(app.FitFunctionEditField.Value));
            app.FitOptions = defaultFitOptions(app, length(coeffs));
        end

        function FitOptionsButtonPushed(app, ~)
            ftype = fittype(app.FitFunctionEditField.Value);
            coeffs = coeffnames(ftype);
            if isempty(app.FitOptions)
                app.FitOptions = defaultFitOptions(app, length(coeffs));
            end
            FitOptionsDialog(app, app.FitOptions, coeffs);
        end        

        % Button pushed function: FitButton
        function FitButtonPushed(app, ~)
            func = app.FunctionDropDown.Value;
            x = app.FitPlot.XData;
            y = app.FitPlot.YData;
            ll = app.rangeLine.Position(1,1);
            rl = app.rangeLine.Position(2,1);
            [i1, ~] = closest(app, x, ll);
            [i2, ~] = closest(app, x, rl);
            x = x(i1:i2);
            y = y(i1:i2);
            switch func
                case 'Exponential'
                    f = 'exp1';
                case 'Polynomial'
                    f = 'poly2';
                case 'Linear'
                    f = 'poly1';
                case 'Custom Function'
                    f = fittype(app.customFunction, ...
                    'dependent',{'y'},'independent',{'x'}, ...
                    'coefficients', coeffnames(fittype(app.customFunction)));
            end
            try
                if ~isempty(app.FitOptions)
                    [fitobject, gof] = fit(x', y', f, app.FitOptions);
                else
                    [fitobject, gof] = fit(x', y', f);
                end
                span = convertStringsToChars('('+string(ll)+','+string(rl)+')');
                newfit = {app.FitNameEditField.Value, app.FitFunctionEditField.Value, gof.sse, gof.rsquare, gof.adjrsquare, span};
                app.FitsTable.Data = [app.FitsTable.Data; newfit];
                app.currentFit = size(app.FitsTable.Data, 1);
                app.FitsTable.RowName{end+1} = app.currentFit;
                app.FitNameEditField.Value = sprintf('unnamed fit %d', app.currentFit);
                app.fits{end+1} = {fitobject; i1; i2};
                plotFit(app);
                removeStyle(app.FitsTable);
                addStyle(app.FitsTable, uistyle('BackgroundColor', '#0072BD', 'FontColor', 'white'), 'row', app.currentFit);
                app.RemoveFitButton.Enable = true;
                app.ShowFitSwitch.Value = 'On';
            catch e 
                app.FitResults.Value = e.message;
            end
        end

        function RemoveFitButtonPushed(app, ~)
            if app.currentFit <= size(app.FitsTable.Data,1)
               app.FitsTable.Data(app.currentFit, :) = [];
               app.fits{app.currentFit} = [];
               app.fits = app.fits(~cellfun(@isempty, app.fits));
            else
               app.FitResults.Value = "Select a fit to remove."; 
            end
        end

        function FitsTableCellSelection(app, event)
            app.selectedCell = event.Indices;
            app.FitsTable.ColumnEditable = [true false false false false false];
            row = app.selectedCell(1);
            if row ~= app.currentFit
                removeStyle(app.FitsTable);
                addStyle(app.FitsTable, uistyle('BackgroundColor', '#0072BD', 'FontColor', 'white'), 'row', row);
                app.currentFit = row;
                plotFit(app)
            end
        end

        % Value changing function: ReflectionSlider
        function ReflectionPointKnobValueChanging(app, event)
            changingValue = event.Value;
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data') && app.ReflectionLineSwitch.Value
                drawReflectionLine(app, changingValue);
            end
        end

        % Value changed function: ReflectionPointKnob
        function ReflectionPointKnobValueChanged(app, ~)
            index = app.DataSelect.Value{3};
            app.DataSelect.ItemsData{index}{2} = getTimeConstant(app, app.DataSelect.Value{1});
            app.DataSelect.Value = app.DataSelect.ItemsData{index};
            if strcmp(app.PlotDropDown.Value, 'Calculated Data')
                set(app.FitPlot, 'XData', app.DataSelect.Value{2}(2,:), 'YData', app.DataSelect.Value{2}(1,:));
            end
            app.reflectionPoints(index) = app.ReflectionPointKnob.Value;
        end

        % Value changing function: VotlageToleranceKnob
        function VoltageToleranceKnobValueChanged(app, ~)
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')
                data = app.DataSelect.Value{1};
                I = abs(data(1,:)) > app.VoltageToleranceKnob.Value;
                data(:,I) = [];
                set(app.FitPlot, 'XData', data(2,:), 'YData', data(1,:));
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                tau = app.DataSelect.Value{2};
                I = abs(tau(1,:)) > app.VoltageToleranceKnob.Value;
                tau(:,I) = [];
                set(app.FitPlot, 'XData', tau(2,:), 'YData', tau(1,:));
            end
        end 

        % Value changed function: FlipDataCheckBox
        function FlipDataButtonPushed(app, ~)
            index = app.DataSelect.Value{3};
            data = app.DataSelect.Value{1};
            tau = app.DataSelect.Value{2};
            flippedData = - data(1,:);
            flippedTau = - tau(1,:);
            data = [flippedData; data(2,:)];
            tau = [flippedTau; tau(2,:)];
            app.DataSelect.ItemsData{index} = {data, tau, index};
            app.DataSelect.Value = app.DataSelect.ItemsData{index};
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')
                set(app.FitPlot, 'YData', flippedData);
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                set(app.FitPlot, 'YData', flippedTau);
            end
        end

        % Button pushed function: ResetViewButton
        function ResetViewButtonPushed(app, ~)
            deltat = get(app.FitPlot, 'XData');
            idx = app.DataSelect.Value{3};
            set(app.FitAxes, 'XLim', [deltat(1) deltat(end)]);
            if strcmp(app.PlotDropDown.Value, 'Probe Voltage Data')     
                set(app.rangeLine, 'Position', [deltat(1), 0; deltat(end), 0]);
                app.pvdLimits(idx, :) = [deltat(1) deltat(end)];
            elseif strcmp(app.PlotDropDown.Value, 'Calculated Data')
                set(app.rangeLine, 'Position', [0, 0; deltat(end), 0]);
                app.fdLimits(idx, :) = [0 deltat(end)];
            end
        end

        % Button pushed function: ResetButton
        function ResetButtonButtonPushed(app, ~)
            test = (app.SaveDataButton.Enable == true);
            app.AquireDataButton.Enable = false;
            app.ResetButton.Enable = false;
            app.SaveDataButton.Enable = false;
            reset(app, app.AWG, app.LockIn);
            app.AquireDataButton.Enable = true;
            app.ResetButton.Enable = true;
            if test 
                app.SaveDataButton.Enable = true;
            end
        end

        % TODO: ADD TEXT FILE TO SAVE WITH PUMPPROBE INFO AND SETTINGS
        % Button pushed function: SaveDataButton
        function SaveDataButtonPushed(app, ~)
            saveLockInData(app);
        end

        % Value changed function: AquireMultipleCheckBox
        function AquireMultipleCheckBoxValueChanged(app, ~)
            value = app.AquireMultipleCheckBox.Value;
            if value
                app.Configurations.Enable = true;
                app.AddConfigurationButton.Enable = true;
                app.RemoveConfigurationButton.Enable = true;
                app.AutomaticExportCheckBox.Value = true;
                app.AutomaticExportCheckBox.Enable = false;
                app.EditDefaultPathButton.Enable = true;
            else
                app.Configurations.Enable = false;
                app.AddConfigurationButton.Enable = false;
                app.RemoveConfigurationButton.Enable = false;
                app.AutomaticExportCheckBox.Value = false;
                app.AutomaticExportCheckBox.Enable = true;
                app.EditDefaultPathButton.Enable = false;
            end
        end

        % Button Pushed Function: AddConfigurationButton
        function AddConfigurationButtonPushed(app, ~)
            app.AddConfigurationButton.Enable = false;
            ppA = app.PumpAmplitudeEditField.Value * app.PumpAmplitudeScale.Value;
            ppE = app.PumpEdgeEditField.Value * app.PumpEdgeScale.Value;
            ppW = app.PumpWidthEditField.Value * app.PumpWidthScale.Value;
            prA = app.ProbeAmplitudeEditField.Value * app.ProbeAmplitudeScale.Value;
            prE = app.ProbeEdgeEditField.Value * app.ProbeEdgeScale.Value;
            prW = app.ProbeWidthEditField.Value * app.ProbeWidthScale.Value;
            prS = app.ProbeSpanEditField.Value * app.ProbeSpanScale.Value;
            config = getConfigName(app);
            app.Configurations.Items{end+1} = config;
            app.Configurations.ItemsData{end+1} = {ppA; ppE; ppW; prA; prE; prW; prS; length(app.Configurations.Items)};
            app.Configurations.Value = app.Configurations.ItemsData{end};
            app.AddConfigurationButton.Enable = true;
            app.RemoveConfigurationButton.Enable = true;
        end

        % Button Pushed Function: RemoveConfigurationButton
        function RemoveConfigurationButtonPushed(app, ~)
            if ~isempty(app.Configurations.Items)
                idx = app.Configurations.Value{8};
                if idx == 1
                    app.Configurations.Items = app.Configurations.Items(2:end);
                    app.Configurations.ItemsData = app.Configurations.ItemsData(2:end);
                if isempty(app.Configurations.Items)
                    app.RemoveConfigurationButton.Enable = false;
                    return;
                end
                    app.Configurations.Value = app.Configurations.ItemsData{1};
                elseif idx == length(app.Configurations.Items) 
                    app.Configurations.Items = app.Configurations.Items(1:end-1);
                    app.Configurations.ItemsData = app.Configurations.ItemsData(1:end-1);
                    if isempty(app.Configurations.Items)
                        app.RemoveConfigurationButton.Enable = false;
                        return;
                    end
                    app.Configurations.Value = app.Configurations.ItemsData{end};
                else
                    app.Configurations.Items = [app.Configurations.Items(1:idx-1) app.Configurations.Items(idx+1:end)];
                    app.Configurations.ItemsData = [app.Configurations.ItemsData(1:idx-1) app.Configurations.ItemsData(idx+1:end)];
                    if isempty(app.Configurations.Items)
                        app.RemoveConfigurationButton.Enable = false;
                        return;
                    end
                    app.Configurations.Value = app.Configurations.ItemsData{idx};
                end
                for k = 1:length(app.Configurations.Items)
                    app.Configurations.ItemsData{k}{8} = k;
                end
            end
        end

        % Value changed function: AutomaticExportCheckBox
        function AutomaticExportCheckBoxValueChanged(app, ~)
            if app.AutomaticExportCheckBox.Value 
                app.EditDefaultPathButton.Enable = true;
            else
                app.EditDefaultPathButton.Enable = false;
            end
        end

        % Button pushed function: EditDefaultPathButton
        function EditDefaultPathButtonPushed(app, ~)
            app.exportPath = uigetdir();
            figure(app.UIFigure);
        end

        % Value changed function: ReflectionLineSwitch
        function ReflectionLineSwitchValueChanged(app, ~)
            if app.ReflectionLineSwitch.Value
                drawReflectionLine(app, app.ReflectionPointKnob.Value);
            else
                delete(app.reflectionLineHandle);
                app.reflectionLineHandle = [];
            end
        end

        function ShowFitSwitchValueChanged(app, ~)
            if strcmp(app.ShowFitSwitch.Value, 'On') && ~isempty(app.fits)
                redrawFitAxes(app);
                plotFit(app);
            else
                redrawFitAxes(app);
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1356 789];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 1 1356 789];

            % Create PumpProbeTab
            app.PumpProbeTab = uitab(app.TabGroup);
            app.PumpProbeTab.Title = 'Pump-Probe';

            % Create AquireDataButton
            app.AquireDataButton = uibutton(app.PumpProbeTab, 'push');
            app.AquireDataButton.ButtonPushedFcn = createCallbackFcn(app, @AquireDataButtonPushed, true);
            app.AquireDataButton.Position = [49 84 262 48];
            app.AquireDataButton.Text = {'Aquire Data'; ''};

            % Create ProbePanel
            app.ProbePanel = uipanel(app.PumpProbeTab);
            app.ProbePanel.Title = 'Probe';
            app.ProbePanel.Position = [48 532 261 164];

            % Create ProbeSpanEditFieldLabel
            app.ProbeSpanEditFieldLabel = uilabel(app.ProbePanel);
            app.ProbeSpanEditFieldLabel.HorizontalAlignment = 'right';
            app.ProbeSpanEditFieldLabel.Position = [38 109 69 22];
            app.ProbeSpanEditFieldLabel.Text = 'Probe Span';

            % Create ProbeSpanEditField
            app.ProbeSpanEditField = uieditfield(app.ProbePanel, 'numeric');
            app.ProbeSpanEditField.Position = [122 109 73 22];
            app.ProbeSpanEditField.Value = 100;

            % Create ProbeAmplitudeEditFieldLabel
            app.ProbeAmplitudeEditFieldLabel = uilabel(app.ProbePanel);
            app.ProbeAmplitudeEditFieldLabel.HorizontalAlignment = 'right';
            app.ProbeAmplitudeEditFieldLabel.Position = [13 77 94 22];
            app.ProbeAmplitudeEditFieldLabel.Text = 'Probe Amplitude';

            % Create ProbeAmplitudeEditField
            app.ProbeAmplitudeEditField = uieditfield(app.ProbePanel, 'numeric');
            app.ProbeAmplitudeEditField.Position = [122 77 73 22];
            app.ProbeAmplitudeEditField.Value = 600;

            % Create ProbeEdgeEditFieldLabel
            app.ProbeEdgeEditFieldLabel = uilabel(app.ProbePanel);
            app.ProbeEdgeEditFieldLabel.HorizontalAlignment = 'right';
            app.ProbeEdgeEditFieldLabel.Position = [38 44 69 22];
            app.ProbeEdgeEditFieldLabel.Text = 'Probe Edge';

            % Create ProbeEdgeEditField
            app.ProbeEdgeEditField = uieditfield(app.ProbePanel, 'numeric');
            app.ProbeEdgeEditField.Position = [122 44 73 22];
            app.ProbeEdgeEditField.Value = 3;

            % Create ProbeWidthEditFieldLabel
            app.ProbeWidthEditFieldLabel = uilabel(app.ProbePanel);
            app.ProbeWidthEditFieldLabel.HorizontalAlignment = 'right';
            app.ProbeWidthEditFieldLabel.Position = [36 11 71 22];
            app.ProbeWidthEditFieldLabel.Text = 'Probe Width';

            % Create ProbeWidthEditField
            app.ProbeWidthEditField = uieditfield(app.ProbePanel, 'numeric');
            app.ProbeWidthEditField.Position = [122 11 73 22];
            app.ProbeWidthEditField.Value = 10;

            % Create ProbeAmplitudeScale
            app.ProbeAmplitudeScale = uidropdown(app.ProbePanel);
            app.ProbeAmplitudeScale.Items = {'mV'};
            app.ProbeAmplitudeScale.ItemsData = 0.001;
            app.ProbeAmplitudeScale.Position = [201 78 51 22];
            app.ProbeAmplitudeScale.Value = 0.001;

            % Create ProbeEdgeScale
            app.ProbeEdgeScale = uidropdown(app.ProbePanel);
            app.ProbeEdgeScale.Items = {'ns'};
            app.ProbeEdgeScale.ItemsData = 1e-09;
            app.ProbeEdgeScale.Position = [201 45 51 22];
            app.ProbeEdgeScale.Value = 1e-09;

            % Create ProbeWidthScale
            app.ProbeWidthScale = uidropdown(app.ProbePanel);
            app.ProbeWidthScale.Items = {'ns'};
            app.ProbeWidthScale.ItemsData = 1e-09;
            app.ProbeWidthScale.Position = [201 12 51 22];
            app.ProbeWidthScale.Value = 1e-09;

            % Create ProbeSpanScale
            app.ProbeSpanScale = uidropdown(app.ProbePanel);
            app.ProbeSpanScale.Items = {'s', 'ms', 'ns'};
            app.ProbeSpanScale.ItemsData = [1, 1e-3, 1e-9];
            app.ProbeSpanScale.Position = [201 110 51 22];
            app.ProbeSpanScale.Value = 1e-9;

            % Create PumpPanel
            app.PumpPanel = uipanel(app.PumpProbeTab);
            app.PumpPanel.Title = 'Pump';
            app.PumpPanel.Position = [49 370 261 144];

            % Create PumpAmplitudeEditFieldLabel
            app.PumpAmplitudeEditFieldLabel = uilabel(app.PumpPanel);
            app.PumpAmplitudeEditFieldLabel.HorizontalAlignment = 'right';
            app.PumpAmplitudeEditFieldLabel.Position = [8 91 94 22];
            app.PumpAmplitudeEditFieldLabel.Text = 'Pump Amplitude';

            % Create PumpAmplitudeEditField
            app.PumpAmplitudeEditField = uieditfield(app.PumpPanel, 'numeric');
            app.PumpAmplitudeEditField.Position = [117 91 78 22];
            app.PumpAmplitudeEditField.Value = 825;

            % Create PumpEdgeEditFieldLabel
            app.PumpEdgeEditFieldLabel = uilabel(app.PumpPanel);
            app.PumpEdgeEditFieldLabel.HorizontalAlignment = 'right';
            app.PumpEdgeEditFieldLabel.Position = [33 56 68 22];
            app.PumpEdgeEditFieldLabel.Text = 'Pump Edge';

            % Create PumpEdgeEditField
            app.PumpEdgeEditField = uieditfield(app.PumpPanel, 'numeric');
            app.PumpEdgeEditField.Position = [116 56 78 22];
            app.PumpEdgeEditField.Value = 3;

            % Create PumpWidthEditFieldLabel
            app.PumpWidthEditFieldLabel = uilabel(app.PumpPanel);
            app.PumpWidthEditFieldLabel.HorizontalAlignment = 'right';
            app.PumpWidthEditFieldLabel.Position = [31 19 71 22];
            app.PumpWidthEditFieldLabel.Text = 'Pump Width';

            % Create PumpWidthEditField
            app.PumpWidthEditField = uieditfield(app.PumpPanel, 'numeric');
            app.PumpWidthEditField.Position = [117 19 78 22];
            app.PumpWidthEditField.Value = 10;

            % Create PumpWidthScale
            app.PumpWidthScale = uidropdown(app.PumpPanel);
            app.PumpWidthScale.Items = {'ns'};
            app.PumpWidthScale.ItemsData = 1e-09;
            app.PumpWidthScale.Position = [200 19 52 22];
            app.PumpWidthScale.Value = 1e-09;

            % Create PumpEdgeScale
            app.PumpEdgeScale = uidropdown(app.PumpPanel);
            app.PumpEdgeScale.Items = {'ns'};
            app.PumpEdgeScale.ItemsData = 1e-09;
            app.PumpEdgeScale.Position = [200 56 52 22];
            app.PumpEdgeScale.Value = 1e-09;

            % Create PumpAmplitudeScale
            app.PumpAmplitudeScale = uidropdown(app.PumpPanel);
            app.PumpAmplitudeScale.Items = {'mV'};
            app.PumpAmplitudeScale.ItemsData = 0.001;
            app.PumpAmplitudeScale.Position = [200 91 52 22];
            app.PumpAmplitudeScale.Value = 0.001;

            % Create LockinPanel
            app.LockinPanel = uipanel(app.PumpProbeTab);
            app.LockinPanel.Title = 'Lock-in';
            app.LockinPanel.Position = [48 247 261 105];

            % Create LockinFrequencyEditFieldLabel
            app.LockinFrequencyEditFieldLabel = uilabel(app.LockinPanel);
            app.LockinFrequencyEditFieldLabel.HorizontalAlignment = 'right';
            app.LockinFrequencyEditFieldLabel.Position = [12 47 104 22];
            app.LockinFrequencyEditFieldLabel.Text = 'Lock-in Frequency';

            % Create LockinFrequencyEditField
            app.LockinFrequencyEditField = uieditfield(app.LockinPanel, 'numeric');
            app.LockinFrequencyEditField.Position = [121 47 74 22];
            app.LockinFrequencyEditField.Value = 1007;

            % Create SamplesEditFieldLabel
            app.SamplesEditFieldLabel = uilabel(app.LockinPanel);
            app.SamplesEditFieldLabel.HorizontalAlignment = 'right';
            app.SamplesEditFieldLabel.Position = [59 15 52 22];
            app.SamplesEditFieldLabel.Text = 'Samples';

            % Create SamplesEditField
            app.SamplesEditField = uieditfield(app.LockinPanel, 'numeric');
            app.SamplesEditField.Position = [121 14 75 22];
            app.SamplesEditField.Value = 800;

            % Create LockInFreqScale
            app.LockInFreqScale = uidropdown(app.LockinPanel);
            app.LockInFreqScale.Items = {'Hz', 'MHz'};
            app.LockInFreqScale.ItemsData = [1, 1e6];
            app.LockInFreqScale.Position = [200 47 52 22];
            app.LockInFreqScale.Value = 1;

            % Create LockinIPEditFieldLabel
            app.LockinIPEditFieldLabel = uilabel(app.PumpProbeTab);
            app.LockinIPEditFieldLabel.HorizontalAlignment = 'right';
            app.LockinIPEditFieldLabel.Position = [46 207 62 22];
            app.LockinIPEditFieldLabel.Text = 'Lock-in IP:';

            % Create LockinIPEditField
            app.LockinIPEditField = uieditfield(app.PumpProbeTab, 'text');
            app.LockinIPEditField.Position = [123 207 186 22];
            app.LockinIPEditField.Value = '169.254.11.17';

            % Create PumpProbeLog
            app.PumpProbeLog = uitextarea(app.PumpProbeTab);
            app.PumpProbeLog.Editable = 'off';
            app.PumpProbeLog.BackgroundColor = [0.651 0.651 0.651];
            app.PumpProbeLog.Position = [877 31 433 218];

            % Create ResetButton
            app.ResetButton = uibutton(app.PumpProbeTab, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonButtonPushed, true);
            app.ResetButton.Enable = 'off';
            app.ResetButton.Position = [48 142 262 48];
            app.ResetButton.Text = 'Reset';

            % Create SaveDataButton
            app.SaveDataButton = uibutton(app.PumpProbeTab, 'push');
            app.SaveDataButton.ButtonPushedFcn = createCallbackFcn(app, @SaveDataButtonPushed, true);
            app.SaveDataButton.Enable = 'off';
            app.SaveDataButton.Position = [49 26 262 48];
            app.SaveDataButton.Text = 'Save Data';

            % Create Configurations
            app.Configurations = uilistbox(app.PumpProbeTab);
            app.Configurations.Items = {};
            app.Configurations.Enable = 'off';
            app.Configurations.Position = [513 87 347 162];
            app.Configurations.Value = {};

            % Create AddConfigurationButton
            app.AddConfigurationButton = uibutton(app.PumpProbeTab, 'push');
			app.AddConfigurationButton.ButtonPushedFcn = createCallbackFcn(app, @AddConfigurationButtonPushed, true);
            app.AddConfigurationButton.Enable = 'off';
            app.AddConfigurationButton.Position = [513 31 168 47];
            app.AddConfigurationButton.Text = 'Add Configuration';

            % Create RemoveConfigurationButton
            app.RemoveConfigurationButton = uibutton(app.PumpProbeTab, 'push');
            app.RemoveConfigurationButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveConfigurationButtonPushed, true);
            app.RemoveConfigurationButton.Enable = 'off';
            app.RemoveConfigurationButton.Position = [698 31 162 47];
            app.RemoveConfigurationButton.Text = 'Remove Configuration';

            % Create OptionsPanel
            app.OptionsPanel = uipanel(app.PumpProbeTab);
            app.OptionsPanel.TitlePosition = 'centertop';
            app.OptionsPanel.Title = 'Options';
            app.OptionsPanel.Position = [337 31 156 220];

            % Create AquireMultipleCheckBox
            app.AquireMultipleCheckBox = uicheckbox(app.OptionsPanel);
            app.AquireMultipleCheckBox.ValueChangedFcn = createCallbackFcn(app, @AquireMultipleCheckBoxValueChanged, true);
            app.AquireMultipleCheckBox.Text = {'Aquire Multiple'};
            app.AquireMultipleCheckBox.Position = [14 168 101 22];

            % Create AutomaticExportCheckBox
            app.AutomaticExportCheckBox = uicheckbox(app.OptionsPanel);
            app.AutomaticExportCheckBox.ValueChangedFcn = createCallbackFcn(app, @AutomaticExportCheckBoxValueChanged, true);
            app.AutomaticExportCheckBox.Text = 'Automatic Export';
            app.AutomaticExportCheckBox.Position = [14 42 114 22];

            % Create DisablePlottingCheckBox
            app.DisablePlottingCheckBox = uicheckbox(app.OptionsPanel);
            app.DisablePlottingCheckBox.Text = 'Disable Plotting';
            app.DisablePlottingCheckBox.Position = [14 136 106 22];

            % Create EditDefaultPathButton
            app.EditDefaultPathButton = uibutton(app.OptionsPanel, 'push');
            app.EditDefaultPathButton.ButtonPushedFcn = createCallbackFcn(app, @EditDefaultPathButtonPushed, true);
            app.EditDefaultPathButton.Enable = 'off';
            app.EditDefaultPathButton.Position = [14 9 121 24];
            app.EditDefaultPathButton.Text = 'Edit Default Path';

            % Create Option3CheckBox
            app.Option3CheckBox = uicheckbox(app.OptionsPanel);
            app.Option3CheckBox.Text = 'Overlay Plots';
            app.Option3CheckBox.Position = [14 104 67 22];

            % Create Option4CheckBox
            app.Option4CheckBox = uicheckbox(app.OptionsPanel);
            app.Option4CheckBox.Text = 'Option 4';
            app.Option4CheckBox.Position = [14 73 67 22];

            % Create SettingDropDownLabel
            app.SettingDropDownLabel = uilabel(app.PumpProbeTab);
            app.SettingDropDownLabel.HorizontalAlignment = 'right';
            app.SettingDropDownLabel.Position = [46 714 46 22];
            app.SettingDropDownLabel.Text = 'Setting:';

            % Create SettingDropDown
            app.SettingDropDown = uidropdown(app.PumpProbeTab);
            app.SettingDropDown.Items = {'Sweep Phase', 'Sweep DC Offset (IV)','Vary Amplitude', 'Vary Width', 'Vary Edge'};
            app.SettingDropDown.Position = [97 714 146 22];
            app.SettingDropDown.Value = 'Sweep Phase';

            % Create EditSettingsButton
            app.EditSettingsButton = uibutton(app.PumpProbeTab, 'push');
            app.EditSettingsButton.ButtonPushedFcn = createCallbackFcn(app, @EditSettingsButtonPushed, true);
            app.EditSettingsButton.Position = [248 714 59 22];
            app.EditSettingsButton.Text = 'Edit';

            % Create SignalAxes
            app.SignalAxes = uiaxes(app.PumpProbeTab);
            title(app.SignalAxes, 'Probe Voltage')
            xlabel(app.SignalAxes, 'Time delay (ns)')
            ylabel(app.SignalAxes, 'Probe Voltage (mV)')
            zlabel(app.SignalAxes, 'Z')
            app.SignalAxes.Toolbar.Visible = 'off';
            app.SignalAxes.XGrid = 'on';
            app.SignalAxes.YGrid = 'on';
            app.SignalAxes.TickDir = 'both';
            app.SignalAxes.Position = [338 280 981 456];
            
            % Create FitDataTab
            app.FitDataTab = uitab(app.TabGroup);
            app.FitDataTab.Title = 'Data Fitting';
            
            % Create FitAxes
            app.FitAxes = uiaxes(app.FitDataTab);
            title(app.FitAxes, 'Data Plot')
            xlabel(app.FitAxes, 'Time delay (ns)')
            ylabel(app.FitAxes, 'Independent variable (V)')
            zlabel(app.FitAxes, 'Z')
            app.FitAxes.Toolbar.Visible = 'off';
            app.FitAxes.XGrid = 'on';
            app.FitAxes.YGrid = 'on';
            app.FitAxes.TickDir = 'both';
            app.FitAxes.Position = [610 302 728 446];
  
            % Create ProbeVoltageDataSelectionPanel
            app.ProbeVoltageDataSelectionPanel = uipanel(app.FitDataTab);
            app.ProbeVoltageDataSelectionPanel.Title = 'Probe Voltage Data Selection';
            app.ProbeVoltageDataSelectionPanel.Position = [19 464 570 284];
            
            % Create DataSelect
            app.DataSelect = uilistbox(app.ProbeVoltageDataSelectionPanel);
            app.DataSelect.Items = {};
            app.DataSelect.ValueChangedFcn = createCallbackFcn(app, @DataSelectValueChanged, true);
            app.DataSelect.Position = [12 11 295 247];
            app.DataSelect.Value = {};
            
            % Create ImportButton
            app.ImportButton = uibutton(app.ProbeVoltageDataSelectionPanel, 'push');
            app.ImportButton.ButtonPushedFcn = createCallbackFcn(app, @ImportButtonPushed, true);
            app.ImportButton.Position = [323 208 237 50];
            app.ImportButton.Text = 'Import';
            
            % Create ExportButton
            app.ExportButton = uibutton(app.ProbeVoltageDataSelectionPanel, 'push');
            app.ExportButton.ButtonPushedFcn = createCallbackFcn(app, @ExportButtonPushed, true);
            app.ExportButton.Position = [322 144 237 49];
            app.ExportButton.Text = 'Export';
            
            % Create GenerateExportFilesCheckBox
            app.GenerateExportFilesCheckBox = uicheckbox(app.ProbeVoltageDataSelectionPanel);
            app.GenerateExportFilesCheckBox.Text = 'Generate Export Files';
            app.GenerateExportFilesCheckBox.Position = [322 106 139 22];
            
            % Create ExportFeedback
            app.ExportFeedback = uitextarea(app.ProbeVoltageDataSelectionPanel);
            app.ExportFeedback.Editable = 'off';
            app.ExportFeedback.HorizontalAlignment = 'center';
            app.ExportFeedback.WordWrap = 'off';
            app.ExportFeedback.Position = [322 11 237 76];
            
            % Create DataAnalysisTypeLabel
            app.DataAnalysisTypeLabel = uilabel(app.FitDataTab);
            app.DataAnalysisTypeLabel.HorizontalAlignment = 'right';
            app.DataAnalysisTypeLabel.Position = [15 428 112 22];
            app.DataAnalysisTypeLabel.Text = 'Data Analysis Type:';
            
            % Create DataAnalysisTypeDropDown
            app.DataAnalysisTypeDropDown = uidropdown(app.FitDataTab);
            app.DataAnalysisTypeDropDown.Items = {'Default'};
            app.DataAnalysisTypeDropDown.Tooltip = {'Choice of data analysis type will change some fitting options and axes controls.'};
            app.DataAnalysisTypeDropDown.Position = [142 428 164 22];
            app.DataAnalysisTypeDropDown.Value = 'Default';

            % Create PlotDropDownLabel
            app.PlotDropDownLabel = uilabel(app.FitDataTab);
            app.PlotDropDownLabel.HorizontalAlignment = 'right';
            app.PlotDropDownLabel.Position = [355 427 30 22];
            app.PlotDropDownLabel.Text = 'Plot:';

            % Create PlotDropDown
            app.PlotDropDown = uidropdown(app.FitDataTab);
            app.PlotDropDown.ValueChangedFcn = createCallbackFcn(app, @PlotDropDownValueChanged, true);
            app.PlotDropDown.Items = {'Probe Voltage Data', 'Calculated Data'};
            app.PlotDropDown.Position = [400 427 189 22];
            app.PlotDropDown.Value = 'Probe Voltage Data';
 
            % Create FitDataPanel
            app.FitDataPanel = uipanel(app.FitDataTab);
            app.FitDataPanel.Enable = 'off';
            app.FitDataPanel.Title = 'Fit Data';
            app.FitDataPanel.Position = [19 20 570 398];
            
            % Create FitNameEditFieldLabel
            app.FitNameEditFieldLabel = uilabel(app.FitDataPanel);
            app.FitNameEditFieldLabel.HorizontalAlignment = 'right';
            app.FitNameEditFieldLabel.Position = [12 342 54 22];
            app.FitNameEditFieldLabel.Text = 'Fit Name';
            
            % Create FitNameEditField
            app.FitNameEditField = uieditfield(app.FitDataPanel, 'text');
            app.FitNameEditField.Position = [78 342 192 22];
            app.FitNameEditField.Value = 'unnamed fit 1';
            
            % Create FunctionLabel
            app.FunctionLabel = uilabel(app.FitDataPanel);
            app.FunctionLabel.HorizontalAlignment = 'right';
            app.FunctionLabel.Position = [7 306 55 22];
            app.FunctionLabel.Text = {'Function:'};
            
            % Create FunctionDropDown
            app.FunctionDropDown = uidropdown(app.FitDataPanel);
            app.FunctionDropDown.ValueChangedFcn = createCallbackFcn(app, @FunctionDropDownValueChanged, true);
            app.FunctionDropDown.Items = {'Exponential', 'Polynomial', 'Linear','Custom Function'};
            app.FunctionDropDown.Position = [77 306 193 22];
            app.FunctionDropDown.Value = 'Exponential';
            
            % Create FitFunctionLabel
            app.FitFunctionLabel = uilabel(app.FitDataPanel);
            app.FitFunctionLabel.HorizontalAlignment = 'right';
            app.FitFunctionLabel.FontSize = 16;
            app.FitFunctionLabel.Position = [17 270 45 22];
            app.FitFunctionLabel.Text = 'y(x) =';

            % Create FitFunctionEditField
            app.FitFunctionEditField = uieditfield(app.FitDataPanel, 'text');
            app.FitFunctionEditField.Position = [77 267 193 25];
            app.FitFunctionEditField.Value = 'a + b*exp(-x/T)';
 
            % Create AutoFitButton
            app.AutoFitButton = uibutton(app.FitDataPanel, 'state');
            app.AutoFitButton.Text = 'Auto Fit';
            app.AutoFitButton.Position = [13 225 121 28];
            
            % Create FitOptionsButton
            app.FitOptionsButton = uibutton(app.FitDataPanel, 'push');
            app.FitOptionsButton.ButtonPushedFcn = createCallbackFcn(app, @FitOptionsButtonPushed, true);
            app.FitOptionsButton.Position = [148 225 121 28];
            app.FitOptionsButton.Text = 'Fit Options';
 
            % Create FitButton
            app.FitButton = uibutton(app.FitDataPanel, 'push');
            app.FitButton.ButtonPushedFcn = createCallbackFcn(app, @FitButtonPushed, true);
            app.FitButton.Position = [12 182 122 30];
            app.FitButton.Text = 'Fit';
            
            % Create RemoveFitButton
            app.RemoveFitButton = uibutton(app.FitDataPanel, 'push');
            app.RemoveFitButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveFitButtonPushed, true);
            app.RemoveFitButton.Enable = 'off';
            app.RemoveFitButton.Position = [148 182 121 30];
            app.RemoveFitButton.Text = 'Remove Fit';
 
            % Create FitResults
            app.FitResults = uitextarea(app.FitDataPanel);
            app.FitResults.Editable = 'off';
            app.FitResults.Position = [288 186 262 178];
            
            % Create FitsTable
            app.FitsTable = uitable(app.FitDataPanel);
            app.FitsTable.CellSelectionCallback = createCallbackFcn(app, @FitsTableCellSelection, true);
            app.FitsTable.ColumnName = {'Fit Name'; 'Fit Function'; 'SSE'; 'R^2'; 'Adj. R^2'; 'Plot Limits'};
            app.FitsTable.RowName = {};
            app.FitsTable.RowStriping = 'off';
            app.FitsTable.ColumnSortable = [true true true true true true];
            app.FitsTable.ColumnEditable = [true false false false false false];
            app.FitsTable.Position = [12 15 547 154];

            % Create AxesControlsPanel
            app.AxesControlsPanel = uipanel(app.FitDataTab);
            app.AxesControlsPanel.Title = 'Axes Controls';
            app.AxesControlsPanel.Position = [610 20 728 272];
            app.AxesControlsPanel.Enable = 'off';
            
            % Create ReflectionPointKnobLabel
            app.ReflectionPointKnobLabel = uilabel(app.AxesControlsPanel);
            app.ReflectionPointKnobLabel.HorizontalAlignment = 'center';
            app.ReflectionPointKnobLabel.Position = [46 76 89 25];
            app.ReflectionPointKnobLabel.Text = {'Tune'; 'Reflection Point'};
            
            % Create ReflectionPointKnob
            app.ReflectionPointKnob = uiknob(app.AxesControlsPanel, 'continuous');
            app.ReflectionPointKnob.ValueChangingFcn = createCallbackFcn(app, @ReflectionPointKnobValueChanging, true);
            app.ReflectionPointKnob.ValueChangedFcn = createCallbackFcn(app, @ReflectionPointKnobValueChanged, true);
            app.ReflectionPointKnob.Position = [51 127 81 81];
            app.ReflectionPointKnob.Limits = [-1 1];
            app.ReflectionPointKnob.Value = 0;
            app.ReflectionPointKnob.MajorTicks = linspace(-1, 1, 5);
            app.ReflectionPointKnob.MajorTickLabels = arrayfun(@num2str, -1:0.5:1, 'UniformOutput', false);
            app.ReflectionPointKnob.Tooltip = {'Tunes the point of reflection on the probe voltage data.'};
            
            % Create VoltageToleranceKnobLabel
            app.VoltageToleranceKnobLabel = uilabel(app.AxesControlsPanel);
            app.VoltageToleranceKnobLabel.HorizontalAlignment = 'center';
            app.VoltageToleranceKnobLabel.Position = [200 76 105 25];
            app.VoltageToleranceKnobLabel.Text = {'Voltage'; 'Tolerance (mV)'};
            
            % Create VoltageToleranceKnob
            app.VoltageToleranceKnob = uiknob(app.AxesControlsPanel, 'continuous');
            app.VoltageToleranceKnob.ValueChangedFcn  = createCallbackFcn(app, @VoltageToleranceKnobValueChanged, true);
            app.VoltageToleranceKnob.Position = [210 130 81 81];
            app.VoltageToleranceKnob.Limits = [0 1];
            app.VoltageToleranceKnob.Value = 1;
            app.VoltageToleranceKnob.MajorTicks = linspace(0, 1, 5);
            app.VoltageToleranceKnob.MajorTickLabels = arrayfun(@num2str, 0:0.25:1, 'UniformOutput', false);
            app.VoltageToleranceKnob.Tooltip = {'Any voltage data outside of the voltage tolerance level will be considered an outlier and removed from the calculated Data.'};
            
            % Create FlipDataButton
            app.FlipDataButton = uibutton(app.AxesControlsPanel, 'push');
            app.FlipDataButton.ButtonPushedFcn = createCallbackFcn(app, @FlipDataButtonPushed, true);
            app.FlipDataButton.Position = [21 14 138 43];
            app.FlipDataButton.Text = 'Flip Data';
            
            % Create ResetViewButton
            app.ResetViewButton = uibutton(app.AxesControlsPanel, 'push');
            app.ResetViewButton.ButtonPushedFcn = createCallbackFcn(app, @ResetViewButtonPushed, true);
            app.ResetViewButton.Position = [182 14 137 43];
            app.ResetViewButton.Text = 'Reset View';
            
            % Create AxesLimitsPanelLabel
            app.VoltageToleranceKnobLabel = uilabel(app.AxesControlsPanel);
            app.VoltageToleranceKnobLabel.HorizontalAlignment = 'center';
            app.VoltageToleranceKnobLabel.Position = [495 215 64 25];
            app.VoltageToleranceKnobLabel.Text = 'Axes Limits';

            % Create AxesLimitsPanel
            app.AxesLimitsPanel = uipanel(app.AxesControlsPanel);
            app.AxesLimitsPanel.TitlePosition = 'centertop';
            app.AxesLimitsPanel.Position = [370 170 318 40];
            app.AxesLimitsPanel.BorderType = 'none';
            
            % Create ReflectionLineSwitchLabel
            app.ReflectionLineSwitchLabel = uilabel(app.AxesControlsPanel);
            app.ReflectionLineSwitchLabel.HorizontalAlignment = 'center';
            app.ReflectionLineSwitchLabel.Position = [368 31 59 26];
            app.ReflectionLineSwitchLabel.Text = {'Reflection'; 'Line'};
            
            % Create ReflectionLineSwitch
            app.ReflectionLineSwitch = uiswitch(app.AxesControlsPanel, 'slider');
            app.ReflectionLineSwitch.ValueChangedFcn = createCallbackFcn(app, @ReflectionLineSwitchValueChanged, true);
            app.ReflectionLineSwitch.Orientation = 'vertical';
            app.ReflectionLineSwitch.Position = [387 84 20 45];
            app.ReflectionLineSwitch.ItemsData = [0, 1];
            
            % Create ShowFitSwitchLabel
            app.ShowFitSwitchLabel = uilabel(app.AxesControlsPanel);
            app.ShowFitSwitchLabel.HorizontalAlignment = 'center';
            app.ShowFitSwitchLabel.Position = [466 23 48 22];
            app.ShowFitSwitchLabel.Text = 'Show Fit';
            
            % Create ShowFitSwitch
            app.ShowFitSwitch = uiswitch(app.AxesControlsPanel, 'slider');
            app.ShowFitSwitch.ValueChangedFcn = createCallbackFcn(app, @ShowFitSwitchValueChanged, true);
            app.ShowFitSwitch.Items = {'Off', 'On'};
            app.ShowFitSwitch.Orientation = 'vertical';
            app.ShowFitSwitch.Position = [480 85 20 45];
            app.ShowFitSwitch.Value = 'Off';
            
            % Create Switch2_3Label
            app.Switch2_3Label = uilabel(app.AxesControlsPanel);
            app.Switch2_3Label.HorizontalAlignment = 'center';
            app.Switch2_3Label.Position = [558 21 48 22];
            app.Switch2_3Label.Text = 'Switch2';
            
            % Create Switch2_3
            app.Switch2_3 = uiswitch(app.AxesControlsPanel, 'slider');
            app.Switch2_3.Items = {'Dont', 'Do'};
            app.Switch2_3.Orientation = 'vertical';
            app.Switch2_3.Position = [572 83 20 45];
            app.Switch2_3.Value = 'Dont';
            
            % Create Switch2_4Label
            app.Switch2_4Label = uilabel(app.AxesControlsPanel);
            app.Switch2_4Label.HorizontalAlignment = 'center';
            app.Switch2_4Label.Position = [639 20 48 22];
            app.Switch2_4Label.Text = 'Switch2';
            
            % Create Switch2_4
            app.Switch2_4 = uiswitch(app.AxesControlsPanel, 'slider');
            app.Switch2_4.Items = {'Dont', 'Do'};
            app.Switch2_4.Orientation = 'vertical';
            app.Switch2_4.Position = [653 82 20 45];
            app.Switch2_4.Value = 'Dont';
            
            % Create ContextMenu
            app.ContextMenu = uicontextmenu(app.UIFigure);
            
            % Assign app.ContextMenu
            app.PumpProbeTab.ContextMenu = app.ContextMenu;
            app.FitDataTab.ContextMenu = app.ContextMenu;

            % Create PreferencesMenu
            app.PreferencesMenu = uimenu(app.ContextMenu);
            app.PreferencesMenu.Text = 'Preferences';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = PumpProbe

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end