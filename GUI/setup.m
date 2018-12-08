function varargout = setup(varargin)
% SETUP MATLAB code for setup.fig
%      SETUP, by itself, creates a new SETUP or raises the existing
%      singleton*.
%
%      H = SETUP returns the handle to a new SETUP or the handle to
%      the existing singleton*.
%
%      SETUP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETUP.M with the given input arguments.
%
%      SETUP('Property','Value',...) creates a new SETUP or raises
%      the existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setup_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setup_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setup

% Last Modified by GUIDE v2.5 03-Dec-2018 12:53:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setup_OpeningFcn, ...
                   'gui_OutputFcn',  @setup_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    fprintf("hello world \n");
    %disp(varargout{:});
    fprintf("\n");
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before setup is made visible.
function setup_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to setup (see VARARGIN)

    % Choose default command line output for setup
    handles.output = hObject;
    handles.camIndex = -1;
    handles.hDataSerialPort = [];
    handles.hControlSerialPort = [];
    handles.wall = struct('left', -6, 'right', 6, 'front', 6, 'back', 0);
    handles.angle = 0;
    handles.cfg = struct('filename', 'mmw_pplcount_demo_default.cfg', 'loaded', 0);
    handles.subzone = [];
    handles.wall_k = 0;
    handles.wall_b = 0;

    % Update handles structure
    guidata(hObject, handles);

    initialize_gui(hObject, handles, false);
    drawRadarRoom(handles);
    %axes1_CreateFcn(hObject, eventdata, handles);
    %axes2_CreateFcn(hObject, eventdata, handles);

    % COM Port Autoconnect comment/uncomment to enable
    %btnConnect_Callback(hObject, eventdata, handles);
    % Set COM Status
    handles = guidata(hObject);
    hCOMStatus = findobj('Tag', 'textCOMStatus');
    if(~isempty(handles.hControlSerialPort) && ~isempty(handles.hDataSerialPort))
        update = 'COM STATUS: Ports connected';
    else
        update = 'COM STATUS: Ports NOT connected';
    end
    set(hCOMStatus,'String', update); 
    
    % connect the port
    controlSerialPort = "COM4";
    dataSerialPort = "COM3";

    % Clear ports
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    % Configure data UART port with input buffer to hold 100+ frames 
    hDataSerialPort = configureDataSport(dataSerialPort, 65536);
    hControlSerialPort = configureControlPort(controlSerialPort);
    handles.hDataSerialPort = hDataSerialPort;
    handles.hControlSerialPort = hControlSerialPort;
    hCOMStatus = findobj('Tag', 'textCOMStatus');
    update = 'COM STATUS: Ports connected';
    set(hCOMStatus,'String', update); 
    guidata(hObject,handles);
    
    
    % UIWAIT makes setup wait for user response (see UIRESUME)
    uiwait(handles.figure1);



% --- Outputs from this function are returned to the command line.
function varargout = setup_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles;

% --- Executes on button press in btnStart.
function btnStart_Callback(hObject, eventdata, handles)
% hObject    handle to btnStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% reopen the serial

controlSerialPort = "COM4"
dataSerialPort = "COM3"

% Clear ports
if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end
% Configure data UART port with input buffer to hold 100+ frames 
hDataSerialPort = configureDataSport(dataSerialPort, 65536);
hControlSerialPort = configureControlPort(controlSerialPort);
handles.hDataSerialPort = hDataSerialPort;
handles.hControlSerialPort = hControlSerialPort;
hCOMStatus = findobj('Tag', 'textCOMStatus');
update = 'COM STATUS: Ports connected';
set(hCOMStatus,'String', update); 
guidata(hObject,handles);



if(length(instrfind('Type','serial', 'Status','open'))>=2 && ~isempty(handles.hControlSerialPort) && ~isempty(handles.hDataSerialPort))
    mmwDemoCliPrompt = char('mmwDemo:/>');
    fprintf("start of function");
    hControlSerialPort = handles.hControlSerialPort;

    % load cfg 
    % Read Chirp Configuration file
    cliCfg = readCfg(handles.cfg.filename);
    [Params cliCfg] = parseCfg(cliCfg, handles.angle); 
    handles.params = Params;
    guidata(hObject, handles);
    %Send CLI configuration to IWR16xx
    fprintf('Sending configuration from %s file to IWR16xx ...\n', handles.cfg.filename);
    disp(hControlSerialPort);
    fprintf("begin to output the control data\n");
    newfile = fopen("a.txt", 'w');
    for k=1:length(cliCfg)
        fprintf(hControlSerialPort, cliCfg{k});
        fprintf(newfile, cliCfg{k});
        fprintf('%s\n', cliCfg{k});
        echo = fgetl(hControlSerialPort); % Get an echo of a command
        done = fgetl(hControlSerialPort); % Get "Done" 
        prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
    end
    fprintf("end\n");
    fclose(hControlSerialPort);
    fclose(newfile);
    delete(hControlSerialPort);
    
    % update output
    %editLW_Callback(findobj('Tag', 'editLW'), eventdata, handles);
    %editRW_Callback(findobj('Tag', 'editRW'), eventdata, handles);
    %editFW_Callback(findobj('Tag', 'editFW'), eventdata, handles);
    %editBW_Callback(findobj('Tag', 'editBW'), eventdata, handles);
    %editAng_Callback(findobj('Tag', 'editAng'), eventdata, handles)
    
    
    
    %setup_OutputFcn(hObject,eventdata,guidata(hObject));
    fprintf("ready to resume");
    disp(handles.wall_k);
    disp(handles.wall_b);
 

    uiresume(gcbf);
else
    warndlg('Error: Can not start COM ports not connected. Please select and connect.');
end

fprintf("aaa");
    


% --- Executes on button press in btnCancel.
function btnCancel_Callback(hObject, eventdata, handles)
% hObject    handle to btnCancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(instrfind('Type','serial', 'Status','open'))
initialize_gui(gcbf, handles, true);
close(gcbf)




% --------------------------------------------------------------------
function initialize_gui(fig_handle, handles, isreset)


% Update handles structure
guidata(handles.figure1, handles);




%{
% --- Executes on button press in pushbuttonBrowse.
function pushbuttonBrowse_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonBrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf("bbb");
% disp(handles.angle);
selectFile = findobj('Tag','radiobuttonSelectFile');
if(selectFile.Value)
     % Get Chirp Config File
     [filename, pathname] = ...
     uigetfile('*.cfg','*.*');
     configurationFileName = [pathname filename];
     handles.cfg.filename = configurationFileName;
     if (filename ~= 0)
        % Read Chirp Configuration file
        cliCfg = readCfg(handles.cfg.filename);
        [Params cliCfg] = parseCfg(cliCfg, handles.angle);
        fprintf("bye");
        % disp(handles.angle);
        handles.params = Params;
        guidata(hObject,handles)
        drawRadarRoom(handles);
     end
     guidata(hObject,handles)
end
%}



% --- Executes on selection change in popupUART.
function popupUART_Callback(hObject, eventdata, handles)
% hObject    handle to popupUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupUART contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupUART

selectIndx = hObject.Value;
menuStrings = hObject.String;
strPort = menuStrings{selectIndx};
comNum = str2num(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);


% --- Executes during object creation, after setting all properties.
function popupUART_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,1};
hObject.UserData = struct('strPort', strPorts{1,1}, 'comNum', numPorts(1,1));




% --- Executes on selection change in popupData.
function popupData_Callback(hObject, eventdata, handles)
% hObject    handle to popupData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupData contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupData

selectIndx = hObject.Value;
menuStrings = hObject.String;
strPort = menuStrings{selectIndx};
comNum = str2num(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);


% --- Executes during object creation, after setting all properties.
function popupData_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,2};
hObject.UserData = struct('strPort', strPorts{1,2}, 'comNum', numPorts(1,2));

% --- Executes on button press in btnConnect.
function btnConnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

hUARTPort = findobj('Tag','editUART');
hDataPort = findobj('Tag','editDATA');

%controlSerialPort = hUARTPort.UserData.strPort;
%dataSerialPort = hDataPort.UserData.strPort;
controlSerialPort = "COM4";
dataSerialPort = "COM3";

% Clear ports
if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end
% Configure data UART port with input buffer to hold 100+ frames 
hDataSerialPort = configureDataSport(dataSerialPort, 65536);
hControlSerialPort = configureControlPort(controlSerialPort);
handles.hDataSerialPort = hDataSerialPort;
handles.hControlSerialPort = hControlSerialPort;
hCOMStatus = findobj('Tag', 'textCOMStatus');
update = 'COM STATUS: Ports connected';
set(hCOMStatus,'String', update); 
guidata(hObject,handles);

%{
function editLW_Callback(hObject, eventdata, handles)
% hObject    handle to editLW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLW as text
%        str2double(get(hObject,'String')) returns contents of editLW as a double
handles.wall.left = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('left wall: %d \n', h.wall.left)
%}

% --- Executes during object creation, after setting all properties.
function editLW_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.left = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);



%{
function editRW_Callback(hObject, eventdata, handles)
% hObject    handle to editRW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editRW as text
%        str2double(get(hObject,'String')) returns contents of editRW as a double
handles.wall.right = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('R wall: %d \n', h.wall.right)
%}

% --- Executes during object creation, after setting all properties.
function editRW_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editRW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.right = str2double(get(hObject,'String'));
guidata(hObject,handles);


%{
function editBW_Callback(hObject, eventdata, handles)
% hObject    handle to editBW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editBW as text
%        str2double(get(hObject,'String')) returns contents of editBW as a double
handles.wall.back = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('back wall: %d \n', h.wall.back)
%}

% --- Executes during object creation, after setting all properties.
function editBW_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editBW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.back = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);


%{
function editFW_Callback(hObject, eventdata, handles)
% hObject    handle to editFW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editFW as text
%        str2double(get(hObject,'String')) returns contents of editFW as a double
handles.wall.front = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('front wall: %d \n', h.wall.front)
%}


% --- Executes during object creation, after setting all properties.
function editFW_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editFW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.front = str2double(get(hObject,'String'));
guidata(hObject,handles);


%{
function editAng_Callback(hObject, eventdata, handles)
% hObject    handle to editAng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAng as text
%        str2double(get(hObject,'String')) returns contents of editAng as a double
handles.angle = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
%}


% --- Executes during object creation, after setting all properties.
function editAng_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
imshow('images/setupfig.jpg','Parent',handles.axes1);
% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in radiobuttonUseDefault.
function radiobuttonUseDefault_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttonUseDefault (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttonUseDefault
if(hObject.Value)
     % Get Chirp Config File
     handles.cfg.filename = 'mmw_pplcount_demo_default.cfg';
     guidata(hObject,handles)
end

function [P, cliCfg] = parseCfg(cliCfg, azimuthTilt)
    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2double(C{3});
            P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                      bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
            P.dataPath.numTxElevAnt = 0;
            P.channelCfg.rxChannelEn = str2double(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'trackingCfg')
            % error check for azimuth tilt
            % if cfg tilt is not same as GUI specified tilt; GUI specified
            % tilt is used instead
            if((str2num(C{8})-90)*pi/180 ~= azimuthTilt)
                temp = cliCfg{k};
                temp(end+1-length(C{8}):end) = '';
                ang = num2str(90-azimuthTilt);
                temp = [temp ang];
                cliCfg{k} = temp;
                fprintf('trackingCfg specifies %d.\n', (str2num(C{8})-90)*pi/180 );
                fprintf('GUI specifies %d. %s will be used for azimuth in cfg.\n',azimuthTilt, ang);
            end
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2double(C{3});
            P.profileCfg.idleTime =  str2double(C{4});
            P.profileCfg.rampEndTime = str2double(C{6});
            P.profileCfg.freqSlopeConst = str2double(C{9});
            P.profileCfg.numAdcSamples = str2double(C{11});
            P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2double(C{2});
            P.frameCfg.chirpEndIdx = str2double(C{3});
            P.frameCfg.numLoops = str2double(C{4});
            P.frameCfg.numFrames = str2double(C{5});
            P.frameCfg.framePeriodicity = str2double(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2double(C{2});
            P.guiMonitor.logMagRange = str2double(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
            P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
    
function [sphandle] = configureDataSport(comPortString, bufferSize)
  
    %comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);
    fprintf("setupConfDataport");

function [sphandle] = configureControlPort(comPortString)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    %comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);
    fprintf("setupConfControlPort");

function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);



% --- Executes during object creation, after setting all properties.
function popupWebcam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupWebcam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
hObject.String = webcamlist();


%{
% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
if(get(hObject,'Value'))
    hWebcamSelect = findobj('Tag', 'popupWebcam');
    handles.camIndex = hWebcamSelect.Value;
    fprintf('webcam enabled %d', hWebcamSelect.Value)
else
    handles.camIndex = -1;
end
guidata(hObject,handles);
%}

function [strPorts numPorts] = get_com_ports()
    
    
    command = 'wmic path win32_pnpentity get caption /format:list | find "COM"';
    [status, cmdout] = system (command);
    UART_COM = regexp(cmdout, 'UART\s+\(COM[0-9]+', 'match');
    UART_COM = (regexp(UART_COM, 'COM[0-9]+', 'match'));
    DATA_COM = regexp(cmdout, 'Data\s+Port\s+\(COM[0-9]+', 'match');
    DATA_COM = (regexp(DATA_COM, 'COM[0-9]+', 'match'));
    
    n = length(UART_COM);
    if (n==0)
        errordlg('Error: No Device Detected')
        return
    else
        CLI_PORT = zeros(n,1);
        S_PORT = zeros(n,1);
        strPorts = {};
        for i=1:n
            temp = cell2mat(UART_COM{1,i});
            strPorts{i,1} = temp;
            CLI_PORT(i,1) = str2num(temp(4:end));
            temp = cell2mat(DATA_COM{1,i});
            strPorts{i,2} = temp;
            S_PORT(i,1) = str2num(temp(4:end));
        end

        CLI_PORT = sort(CLI_PORT);
        S_PORT = sort(S_PORT);
        numPorts = [CLI_PORT, S_PORT];
    end

%{
% --- Executes on button press in checkboxSubzones.
function checkboxSubzones_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxSubzones (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxSubzones
if(~get(hObject,'Value'))
    handles.subzone = [];
    guidata(hObject,handles);
    drawRadarRoom(handles)
end
%}

%{
function editBox_Callback(hObject, eventdata, handles)
% hObject    handle to editBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editBox as text
%        str2double(get(hObject,'String')) returns contents of editBox as a double
enableSubzones = get(findobj('Tag', 'checkboxSubzones'),'Value');
if(enableSubzones)
    boxes = str2num(get(hObject,'String'));
    handles.subzone = boxes;
    guidata(hObject,handles);
    drawRadarRoom(handles)
end
%}

% --- Executes during object creation, after setting all properties.
function editBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function drawRadarRoom(handles)
ax = handles.axes1;
if ishandle(ax)
    children = get(ax, 'children');
    delete(children);
    scene.azimuthTilt = handles.angle*pi/180;
    wall = handles.wall;
    %sensor parameters
    if(isfield(handles, 'params'))
        sensor.rangeMax = floor(handles.params.dataPath.numRangeBins*handles.params.dataPath.rangeResolutionMeters);
    else 
        sensor.rangeMax = 6;
    end
    sensor.rangeMin = 1;
    sensor.azimuthFoV = 120*pi/180; %120 degree FOV in horizontal direction
    sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);
    %hold(ax, 'on')
    plot(ax, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-r'); 
    plot(ax, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-r');
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    margin = 0.5; %[m]
    scene.maxPos = [scene.areaBox(1)-margin ...
                    scene.areaBox(1)+scene.areaBox(3)+margin ...
                    scene.areaBox(2)-margin ...
                    scene.areaBox(2)+scene.areaBox(4)+margin];
    ax.DataAspectRatio = [1 1 1];
    axis(ax, scene.maxPos);
    ax.CameraUpVector = [0,-1, 0];
    grid(ax, 'on');
    grid(ax, 'minor');                
    title(ax, 'Top Down View of Scene');

    % draw wall box
    rectangle(ax, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
end

% draw target box
colors='brgcm';
numBoxes = size(handles.subzone,1);
for nBoxes = 1:numBoxes
    hTargetBoxHandle(nBoxes)= rectangle('Parent', ax, 'Position', handles.subzone(nBoxes,:), 'EdgeColor', colors(nBoxes), 'LineWidth', 4);
end



function editUART_Callback(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editUART as text
%        str2double(get(hObject,'String')) returns contents of editUART as a double
strPort = get(hObject, 'String');
comNum = str2double(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);


% --- Executes during object creation, after setting all properties.
function editUART_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,1};
hObject.UserData = struct('strPort', strPorts{1,1}, 'comNum', numPorts(1,1))



function editDATA_Callback(hObject, eventdata, handles)
% hObject    handle to editDATA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDATA as text
%        str2double(get(hObject,'String')) returns contents of editDATA as a double
strPort = get(hObject, 'String');
comNum = str2double(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);


% --- Executes during object creation, after setting all properties.
function editDATA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDATA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,2};
hObject.UserData = struct('strPort', strPorts{1,2}, 'comNum', numPorts(1,2));

function [sphandle] = configureDataSportMain(comPortNum, bufferSize)
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);
    fprintf("setupMainconfDatasport");

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end


    
    
% --- Executes on button press in pushbutton6.
% callback for init
function pushbutton6_Callback(hObject, eventdata, handles)
fprintf("aaabbb");
controlSerialPort = "COM4";
dataSerialPort = "COM3";
% Clear ports
if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end

hDataSerialPort = configureDataSport(dataSerialPort, 65536);
hControlSerialPort = configureControlPort(controlSerialPort);

fprintf("aaa");
trackerRun = 'Target';
colors='brgcm';
labelTrack = 0;

configurationFileName = 'mmw_pplcount_demo_default.cfg';   
cliCfg = readCfg(configurationFileName);
Params = parseCfg(cliCfg,0);
    

%sensor parameters
%sensor.rangeMax = 6;
sensor.rangeMax = Params.dataPath.numRangeBins*Params.dataPath.rangeIdxToMeters;
sensor.rangeMin = 1;
sensor.azimuthFoV = 120*pi/180; %120 degree FOV in horizontal direction
sensor.framePeriod = Params.frameCfg.framePeriodicity;
sensor.maxURadialVelocity = 20;
sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);

hTargetBoxHandle = [];
peopleCountTotal = 0;
% peopleCountInBox = zeros(1, scene.numberOfTargetBoxes);
peopleCountInBox = zeros(1, size(4, 1));
rxData = zeros(10000,1,'uint8');

maxNumTracks = 20;
maxNumPoints = 250;

hPlotCloudHandleAll = [];
hPlotCloudHandleOutRange = [];
hPlotCloudHandleClutter = [];
hPlotCloudHandleStatic = [];
hPlotCloudHandleDynamic =[];
hPlotPoints3D = [];

clutterPoints = zeros(2,1);
activeTracks = zeros(1, maxNumTracks);

trackingHistStruct = struct('tid', 0, 'allocationTime', 0, 'tick', 0, 'posIndex', 0, 'histIndex', 0, 'sHat', zeros(1000,6), 'ec', zeros(1000,9),'pos', zeros(100,2), 'hMeshU', [], 'hMeshG', [], 'hPlotAssociatedPoints', [], 'hPlotTrack', [], 'hPlotCentroid', []);
trackingHist = repmat(trackingHistStruct, 1, maxNumTracks);


%% Data structures
syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');

frameHeaderStructType = struct(...
    'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
    'version',          {'uint32', 4}, ...
    'platform',         {'uint32', 4}, ...
    'timestamp',        {'uint32', 4}, ... % 600MHz clocks
    'packetLength',     {'uint32', 4}, ... % In bytes, including header
    'frameNumber',      {'uint32', 4}, ... % Starting from 1
    'subframeNumber',   {'uint32', 4}, ...
    'chirpMargin',      {'uint32', 4}, ... % Chirp Processing margin, in ms
    'frameMargin',      {'uint32', 4}, ... % Frame Processing margin, in ms
    'uartSentTime' ,    {'uint32', 4}, ... % Time spent to send data, in ms
    'trackProcessTime', {'uint32', 4}, ... % Tracking Processing time, in ms
    'numTLVs' ,         {'uint16', 2}, ... % Number of TLVs in thins frame
    'checksum',         {'uint16', 2});    % Header checksum

tlvHeaderStruct = struct(...
    'type',             {'uint32', 4}, ... % TLV object Type
    'length',           {'uint32', 4});    % TLV object Length, in bytes, including TLV header 

% Point Cloud TLV object consists of an array of points. 
% Each point has a structure defined below
pointStruct = struct(...
    'range',            {'float', 4}, ... % Range, in m
    'angle',            {'float', 4}, ... % Angel, in rad
    'doppler',          {'float', 4}, ... % Doplper, in m/s
    'snr',              {'float', 4});    % SNR, ratio
% Target List TLV object consists of an array of targets. 
% Each target has a structure define below
targetStruct = struct(...
    'tid',              {'uint32', 4}, ... % Track ID
    'posX',             {'float', 4}, ... % Target position in X dimension, m
    'posY',             {'float', 4}, ... % Target position in Y dimension, m
    'velX',             {'float', 4}, ... % Target velocity in X dimension, m/s
    'velY',             {'float', 4}, ... % Target velocity in Y dimension, m/s
    'accX',             {'float', 4}, ... % Target acceleration in X dimension, m/s2
    'accY',             {'float', 4}, ... % Target acceleration in Y dimension, m/s
    'EC',               {'float', 9*4}, ... % Tracking error covariance matrix, [3x3], in range/angle/doppler coordinates
    'G',                {'float', 4});    % Gating function gain

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
pointLengthInBytes = lengthFromStruct(pointStruct);
targetLengthInBytes = lengthFromStruct(targetStruct);
indexLengthInBytes = 1;

exitRequest = 0;
lostSync = 0;
gotHeader = 0;
outOfSyncBytes = 0;
runningSlow = 0;
maxBytesAvailable = 0;
point3D = [];

frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', [], 'done', 0, ...
    'pointCloud', [], 'targetList', [], 'indexArray', []);
fHist = repmat(frameStatStruct, 1, 10000);
%videoFrame = struct('cdata',[],'colormap', []);
%F = repmat(videoFrame, 10000,1);
optimize = 1;
skipProcessing = 0;
frameNum = 1;
frameNumLogged = 1;
fprintf('------------------\n');

update = 0;

positionAll = [];


while(isvalid(hDataSerialPort))
    h = waitbar(0, 'initializing progress', 'Name', 'detecting the walls...');
    counting = 210;
    while(lostSync == 0 && isvalid(hDataSerialPort) && counting ~= 0)
        counting = counting - 1;
        waitbar(1/500 * (500-counting));
        frameStart = tic;
        fHist(frameNum).timestamp = frameStart;
        bytesAvailable = get(hDataSerialPort,'BytesAvailable');
        if(bytesAvailable > maxBytesAvailable)
            maxBytesAvailable = bytesAvailable;
        end

        fHist(frameNum).bytesAvailable = bytesAvailable;
        if(gotHeader == 0)
            %Read the header first
            [rxHeader, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes, 'uint8');
        end
        fHist(frameNum).start = 1000*toc(frameStart);

        magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
        if(magicBytes ~= syncPatternUINT64)
            reason = 'No SYNC pattern';
            lostSync = 1;
            break;
        end
        if(byteCount ~= frameHeaderLengthInBytes)
            reason = 'Header Size is wrong';
            lostSync = 1;
            break;
        end        
        if(validateChecksum(rxHeader) ~= 0)
            reason = 'Header Checksum is wrong';
            lostSync = 1;
            break; 
        end

        
        frameHeader = readToStruct(frameHeaderStructType, rxHeader);

        if(gotHeader == 1)
            if(targetFrameNum && frameHeader.frameNumber > targetFrameNum)
                targetFrameNum = frameHeader.frameNumber;
                %disp(['Found sync at frame ',num2str(targetFrameNum),'(',num2str(frameNum),'), after ', num2str(1000*toc(lostSyncTime),3), 'ms']);
                gotHeader = 0;
            else
                reason = 'Old Frame';
                gotHeader = 0;
                lostSync = 1;
                break;
            end
        end

        
        % We have a valid header/
        targetFrameNum = frameHeader.frameNumber;
        fHist(frameNum).targetFrameNum = targetFrameNum;
        fHist(frameNum).header = frameHeader;
        
        dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;
        
        fHist(frameNum).bytes = dataLength; 
        numInputPoints = 0;
        numTargets = 0;
        mIndex = [];
        %disp(dataLength);
        

        if(dataLength > 0)
            %Read all packet
            [rxData, byteCount] = fread(hDataSerialPort, double(dataLength), 'uint8');
            if(byteCount ~= double(dataLength))
                reason = 'Data Size is wrong'; 
                lostSync = 1;
                break;  
            end
            offset = 0;
    
            fHist(frameNum).benchmarks(1) = 1000*toc(frameStart);

            % TLV Parsing
%             print(glkessjg); doesn't print anything
            for nTlv = 1:frameHeader.numTLVs
                tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                if(tlvLength + offset > dataLength)
                    reason = 'TLV Size is wrong';
                    lostSync = 1;
                    break;                    
                end
                offset = offset + tlvHeaderLengthInBytes;
                valueLength = tlvLength - tlvHeaderLengthInBytes;
                switch(tlvType)
                    case 6
                        % Point Cloud TLV
                        numInputPoints = valueLength/pointLengthInBytes;
                        if(numInputPoints > 0)                        
                            % Get Point Cloud from the sensor
                            p = typecast(uint8(rxData(offset+1: offset+valueLength)),'single');

                            pointCloud = reshape(p,4, numInputPoints);    
%                            pointCloud(2,:) = pointCloud(2,:)*pi/180;

                            posAll = [pointCloud(1,:).*sin(pointCloud(2,:)); pointCloud(1,:).*cos(pointCloud(2,:))];
                            snrAll = pointCloud(4,:);
                            if (counting <= 200)
                                positionAll = [positionAll posAll];
                            end
                            % disp(posAll);
                            % disp(posAll);
                            % Define wall [BLx BLy W H]
                            scene.areaBox = [-6 0 12 6];

                            % Remove out of Range, Behind the Walls, out of FOV points
                            inRangeInd = (pointCloud(1,:) > 1) & (pointCloud(1,:) < 6) & ...
                                (pointCloud(2,:) > -50*pi/180) &  (pointCloud(2,:) < 50*pi/180) & ...
                                (posAll(1,:) > scene.areaBox(1)) & (posAll(1,:) < (scene.areaBox(1) + scene.areaBox(3))) & ...
                                (posAll(2,:) > scene.areaBox(2)) & (posAll(2,:) < (scene.areaBox(2) + scene.areaBox(4)));
                            pointCloudInRange = pointCloud(:,inRangeInd);
                            posInRange = posAll(:,inRangeInd);
%{
                            % Clutter removal
                            staticInd = (pointCloud(3,:) == 0);        
                            clutterInd = ismember(pointCloud(1:2,:)', clutterPoints', 'rows');
                            clutterInd = clutterInd' & staticInd;
                            clutterPoints = pointCloud(1:2,staticInd);
                            pointCloud = pointCloud(1:3,~clutterInd);
%}
                            numOutputPoints = size(pointCloud,2);   
                            % fprintf("num is %d", numOutputPoints);
                        end                        
                        offset = offset + valueLength;
                        
                                            
                    case 7
                        % Target List TLV
                        numTargets = valueLength/targetLengthInBytes;                        
                        TID = zeros(1,numTargets);
                        S = zeros(6, numTargets);
                        EC = zeros(9, numTargets);
                        G = zeros(1,numTargets);                        
                        for n=1:numTargets
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                            S(:,n)  = typecast(uint8(rxData(offset+5:offset+28)),'single');     %6x4=24bytes
                            EC(:,n) = typecast(uint8(rxData(offset+29:offset+64)),'single');    %9x4=36bytes
                            G(n)    = typecast(uint8(rxData(offset+65:offset+68)),'single');    %1x4=4bytes
                            offset = offset + 68;
                        end
                        %disp(S);
                        
                    case 8
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
                end
            end
            
        end
        
        
        if(numInputPoints == 0)
            numOutputPoints = 0;
            pointCloud = single(zeros(4,0));
            posAll = [];
            posInRange = [];  
        end
        if(numTargets == 0)
            TID = [];
            S = [];
            EC = [];
            G = [];
        end
        
        disp(counting);
    end
%     print(wha); printing the large matrix
    
    
    
    while(lostSync)
        for n=1:8
            [rxByte, byteCount] = fread(hDataSerialPort, 1, 'uint8');
            if(rxByte ~= syncPatternUINT8(n))
                outOfSyncBytes = outOfSyncBytes + 1;
                break;
            end
        end
        if(n == 8)
            lostSync = 0;
            frameNum = frameNum + 1;
            if(frameNum > 10000)
                frameNum = 1;
            end
            
            [header, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
            rxHeader = [syncPatternUINT8'; header];
            byteCount = byteCount + 8;
            gotHeader = 1;
        end
    end
    break;
end
fprintf("finally ends");

%pause(1);
disp(positionAll);
waitbar(1);
delete(h);


ax = handles.axes1;
if ishandle(ax)
    children = get(ax, 'children');
    delete(children);
    scene.azimuthTilt = handles.angle*pi/180;
    wall = handles.wall;
    %sensor parameters
    if(isfield(handles, 'params'))
        sensor.rangeMax = floor(handles.params.dataPath.numRangeBins*handles.params.dataPath.rangeResolutionMeters);
    else 
        sensor.rangeMax = 6;
    end
    sensor.rangeMin = 1;
    sensor.azimuthFoV = 120*pi/180; %120 degree FOV in horizontal direction
    sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);
    %hold(ax, 'on')
    plot(ax, positionAll(1,:),positionAll(2,:),'.k');
    
    hold(ax,'on');
    plot(ax, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-r'); 
    plot(ax, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-r');
    %plot(ax, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), '-b');
    % b = regress(positionAll(2),positionAll(1));
    %plot(ax, X, X*b(1)+b(2),'r');
    [a, error_one_line] = polyfit(positionAll(1,:), positionAll(2,:),1);
    fprintf("first error is");
    disp(error_one_line);
    fprintf("finish");
    line(ax, [-6 6], [-6*a(1)+a(2),6*a(1)+a(2)],'Color', 'blue', 'LineWidth', 3);
    
    
    
    
    plusfun = @(x) max(x,0);
    P0 = [0.1 0 0.0];
    model = @(P,x) (-P(1))*plusfun(P(2)-x) + ((-1)/P(1))*plusfun(x-P(2)) + P(3);
    %{
    x = [0:0.01:2];
y = x * 2;
x1 = [2:0.01:4];
y1 = x1 * (-0.5) + 5;
x = cat(2,x,x1);
y = cat(2,y,y1);
%}
    x = double(positionAll(1,:));
    y = double(positionAll(2,:));
    %ositionAll(1,:);
    %y = positionAll(2,:);
    Pfit = lsqcurvefit(model,P0,x,y);
    x1 = [-6:0.1:6];
    modelpred = model(Pfit,x1);
    opts = statset('nlinfit');
    opts.RobustWgtFun = 'bisquare';
    beta = nlinfit(x,y,model,P0,opts);
    new_beta = model(beta, sort(x1));
    %plot(x, y, 'o', sort(x), new_beta,'r-');
    
    
    plot(ax, x1,modelpred,'r-')
    plot(ax, x1, new_beta, 'k-')
    
    hold(ax,'off');
    %plot(ax, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-r');
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    margin = 0.5; %[m]
    scene.maxPos = [scene.areaBox(1)-margin ...
                    scene.areaBox(1)+scene.areaBox(3)+margin ...
                    scene.areaBox(2)-margin ...
                    scene.areaBox(2)+scene.areaBox(4)+margin];
    ax.DataAspectRatio = [1 1 1];
    axis(ax, scene.maxPos);
    ax.CameraUpVector = [0,-1, 0];
    grid(ax, 'on');
    grid(ax, 'minor');                
    title(ax, 'Top Down View of Scene');

    % draw wall box
    rectangle(ax, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
end

handles = guidata(hObject);
    
handles.wall_k = a(1);
handles.wall_b = a(2);
fprintf("wall_k, b is changed");
guidata(hObject, handles);



wall_distance = (abs(handles.wall_b) / abs(handles.wall_k)) / 10 ;
wall_dis_string = "the distance to wall is " + num2str(wall_distance) + "m";
msgbox(cellstr(wall_dis_string), char('w'));





function CS = validateChecksum(header)
    h = typecast(uint8(header),'uint16');
    a = uint32(sum(h));
    b = uint16(sum(typecast(a,'uint16')));
    CS = uint16(bitcmp(b));





function [sphandle] = configureControlPortMain(comPortNum)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);
    fprintf("setup_main_confcontrolport");



function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end



% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
