%% Lock-in setup
% establish tcpip connection

IP = '169.254.11.17';  % IP address for lockin

t = tcpip(IP,50000);

t.InputBufferSize = 50000;

fopen(t); % open connection

%%
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

%% AWG setup

% establish visa USB connection 

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

fopen(v);

%%
fprintf(v, '*RST'); % reset AWG
fprintf (t, '*CLS'); % clear AWG

%%
fprintf(v, 'SOURce1:FUNCTION SIN');
fprintf(v, 'SOURce1:FREQ 200');
fprintf(v, 'SOURce1:VOLT 0.05');
fprintf(v, 'SOURce1:VOLT:OFFSet 0');
fprintf(v, 'OUTPUT1 ON');
fprintf(t, 'X.'); % read value from Lock-in

 %% Pump probe measurements 

nbs = 1000; % number of steps
offsetMax = 3;
deltaRange = linspace(0, offsetMax, nbs);

hh = gca;
data=zeros(1,nbs); % initialize values
h = plot(hh,deltaRange(1),data(1),'.-','MarkerSize',13,'LineWidth',1.5,'XDataSource','deltaRange','YDataSource','data','Color',[0.4 0.1 1]);
xlabel(hh,'Voltage (V)')
ylabel(hh,'Current (mA)')
chars = ['c', '#', '"'];
for i=1:length(deltaRange)
    c = sprintf('SOURce1:VOLT:OFFSet %g', deltaRange(i));
    fprintf(v, c);
    fprintf(v,'*WAI');
    fprintf(t, 'X.'); % read value from Lock-in
    val = fscanf(t);
    for k = chars
        test = strsplit(val, k);
        n = max(size(test));
        val = test{n};
    end
    x = sscanf(val,'%f'); % correctly converts string to fp number
    if x == 3
       continue; 
    end
    try 
        data(i) = -x;
    catch e 
%         deltaRange(i) = [];
        disp(e.message);
        return;
    end
    set(h, "XData", deltaRange(1:i), "YData", data(1:i));
    pause(0.1);
end
writematrix([data; deltaRange], myFilename); % save data to csv file.
saveas(gcf, myFilename);