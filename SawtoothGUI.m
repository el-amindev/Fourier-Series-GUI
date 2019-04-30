function varargout = SawtoothGUI(varargin)
% SAWTOOTHGUI MATLAB code for SawtoothGUI.fig
%      SAWTOOTHGUI, by itself, creates a new SAWTOOTHGUI or raises the existing
%      singleton*.
%
%      H = SAWTOOTHGUI returns the handle to a new SAWTOOTHGUI or the handle to
%      the existing singleton*.
%
%      SAWTOOTHGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SAWTOOTHGUI.M with the given input arguments.
%
%      SAWTOOTHGUI('Property','Value',...) creates a new SAWTOOTHGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SawtoothGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SawtoothGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SawtoothGUI

% Last Modified by GUIDE v2.5 30-Apr-GUI 18:54:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SawtoothGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SawtoothGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SawtoothGUI is made visible.
function SawtoothGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SawtoothGUI (see VARARGIN)

% Choose default command line output for SawtoothGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SawtoothGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SawtoothGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

X=str2double(get(handles.edit1,'String')); 
T=str2double(get(handles.edit2,'String')); 
H=10;


fs = 100000; 
t = -2*T:1/fs:2*T;
Vm=X/2;


x1 = (Vm)*sawtooth(2*pi*t/T)+Vm;


plot(t,x1,'g','LineWidth',1.5,'Parent',handles.axes1);


axes(handles.axes1);
axis([-2*T-(0.05*T) 2*T+(0.15*T) 0 X+(0.02*X)]);


xlabel('Time','FontSize',10);
ylabel('Amplitude','FontSize',10);
title('Sawtooth','FontSize',10);


syms t n 
a0 = int(((X/T)*t),t,0,T)/T; 
c0 = a0;

w0 = 2*pi/T; 
cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); 


k = 1:H;

c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T);
c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1);


c1';
c2';
mgc1= abs(c1);
mgc2= abs(c2);
axes(handles.axes2);
stem(0,a0);
hold on
stem(-k,mgc2);
hold on
stem(k,mgc1); 


axes(handles.axes3);
zp=[c1];
p=angle(double(zp));
phasep= (p*180/pi);

zn=[c2];
pn=angle(double(zn));
phasen= (p*180/pi)*(-1);

stem(0,0);
hold on
stem(-k,phasen);
hold on
stem(k,phasep);
hold on


dat(1,:)={0,abs(-X),0};

for n = 0;
        an = (X/2);
        bn = atan((X/((n)*pi))/0);               
        bn = radtodeg(bn);
        dat(1,:) = {n an bn};
    for n = 1:1:H


    an = abs(i*X/(2*pi*(n)));
    bn = atan((X/((n)*pi))/0);               
    bn = radtodeg(bn);
    dat(n+1,:) = {n an bn};
    end
end

cnames = {'n','Amplitude','Phase'};
set(handles.uitable1,'data',dat,'ColumnName',cnames);



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


axes(handles.axes4)
N = get(handles.popupmenu1,'value');
X=str2double(get(handles.edit1,'String')); 
T=str2double(get(handles.edit2,'String')); 

wo = 2*pi;                          
c0 = 1/2;                           
t = -5:0.01:5;                    

yt = c0*ones(size(t));             

for n = 1:1:N                    
  cn = -1/(1i*n*wo);                
  yt = yt + 2*abs(cn)*cos(n*wo*t+angle(cn));  
end

plot([-3 -2 -2 -1 -1  0 0 1  1  2 2 3],...    
     [0  1 0 1 0 1 0 1 0 1 0 1], ':');
hold;                              
plot(t,yt);
xlabel('t (seconds)'); ylabel('y(t)');
ttle = ['THIS IS GIBBS! Fourier Series with N =  ',...
         handles.popupmenu1(N)];  %Fix this.
title(ttle);
hold;


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


X=str2double(get(handles.edit1,'String')); 
T=str2double(get(handles.edit2,'String')); 
S=str2double(get(handles.edit3,'String')); 


fs = 100000; 
t = -2*T/S:1/fs:2*T/S;
Vm=X/2;


x1 = (Vm)*sawtooth(2*pi*t*S/T)+Vm;


plot(t,x1,'g','LineWidth',1.5,'Parent',handles.axes1);


axes(handles.axes1);
axis([-2*T-(0.05*T) 2*T+(0.15*T) 0 X+(0.02*X)]);


xlabel('Time','FontSize',10);
ylabel('Amplitude','FontSize',10);
title('Sawtooth','FontSize',10);


syms t n 
a0 = int(((X/T)*t),t,0,T)/T; 
c0 = a0; 

w0 = 2*pi/T; 
cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); 

k = 1:H;

c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T);
c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1); 

c1';
c2';
mgc1= abs(c1);
mgc2= abs(c2);
axes(handles.axes2);
stem(0,a0);
hold on
stem(-k,mgc2);
hold on
stem(k,mgc1); 


axes(handles.axes3);
zp=[c1];
p=angle(double(zp));
phasep= (p*180/pi);

zn=[c2];
pn=angle(double(zn));
phasen= (p*180/pi)*(-1);

stem(0,0);
hold on
stem(-k,phasen); 
hold on
stem(k,phasep); 
hold on


dat(1,:)={0,abs(-X),0};
f = figure('Position',[200 200 400 150]);
for n = 1:1:H
an = abs(-X/((n)*pi));
bn = atan((X/((n)*pi))/0);               
bn = radtodeg(bn);
dat(n+1,:) = {n an bn};
end
cnames = {'n','Amplitude','Phase'};
t = uitable('Parent',f,'Data',dat,'ColumnName',cnames,... 
            'Position',[20 20 360 100]);
txt_title = uicontrol('Style', 'text', 'Position', [20 20 360 100],...
            'String','My Example Title');






% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    url = 'https://github.com/AminSpek/Fourier-Series-GUI';
    web(url,'-browser')
