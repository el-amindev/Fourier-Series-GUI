function varargout = Sawtooth2019(varargin)
% SAWTOOTH2019 MATLAB code for Sawtooth2019.fig
%      SAWTOOTH2019, by itself, creates a new SAWTOOTH2019 or raises the existing
%      singleton*.
%
%      H = SAWTOOTH2019 returns the handle to a new SAWTOOTH2019 or the handle to
%      the existing singleton*.
%
%      SAWTOOTH2019('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SAWTOOTH2019.M with the given input arguments.
%
%      SAWTOOTH2019('Property','Value',...) creates a new SAWTOOTH2019 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Sawtooth2019_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Sawtooth2019_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Sawtooth2019

% Last Modified by GUIDE v2.5 01-May-2019 14:04:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Sawtooth2019_OpeningFcn, ...
                   'gui_OutputFcn',  @Sawtooth2019_OutputFcn, ...
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


% --- Executes just before Sawtooth2019 is made visible.
function Sawtooth2019_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Sawtooth2019 (see VARARGIN)

% Choose default command line output for Sawtooth2019
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Sawtooth2019 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Sawtooth2019_OutputFcn(hObject, eventdata, handles) 
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
% edit1 = Amplitude

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
% edit2 = Period

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
% pushbutton1 = Plot

axes(handles.axes2); % Make averSpec the current axes.
cla reset; % Do a complete and total reset of the axes.

%---------------- to read the value -----------------------&
X=str2double(get(handles.edit1,'String')); %Amplitude
T=str2double(get(handles.edit2,'String')); %Period
H=10; %Harmonics

%---------------- set t, Vm and f -----------------------&
fs = 100000; 
t = -3*T:1/fs:3*T;
Vm=X/2;


%---------------- The equation of the signal -----------------------&
x1 = (Vm)*sawtooth(2*pi*t/T)+Vm; %Sawtooth graph formula


%---------------- plot graph at axes1 -----------------------&
plot(t,x1,'g','LineWidth',1.5,'Parent',handles.axes1);


%----------- set the location of the graph at axes1 ---------------&
axes(handles.axes1);
axis([-3*T-(0.05*T) 3*T+(0.15*T) 0 X+(0.02*X)]);


%---------------- label axis and title -----------------------&
xlabel('Time','FontSize',10);
ylabel('Amplitude','FontSize',10);
title('Sawtooth','FontSize',10);


%---------------- Integrate part -----------------------&
%syms t n % let the equation in term of (t Vm pi T n)
%a0 = int(((X/T)*t),t,0,T)/T; % a0 Formula
%c0 = a0; %Little bit declaration

%w0 = 2*pi/T; % w0 Formula
%cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); % cn Formula


%set k where k equal to n. because n been used in the syms so we could not
%let n = to value
%k = 1:H;

%c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T); % +ve cn
%c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1); % -ve cn


%------------ plot the stem graph for amplitude -------------------&
%c1';
%c2';
%mgc1= abs(c1);
%mgc2= abs(c2);
%axes(handles.axes2);
%stem(0,a0);
%hold on
%stem(-k,mgc2); % -ve n
%hold on
%stem(k,mgc1); % +ve n
    
%---------------- set the phase angle for the graph ---------------------&
%axes(handles.axes3);
%zp=[c1];
%p=angle(double(zp));
%phasep= (p*180/pi);

%zn=[c2];
%pn=angle(double(zn));
%phasen= (p*180/pi)*(-1);

%stem(0,0);
%hold on
%stem(-k,phasen); %-ve n
%hold on
%stem(k,phasep); %+ve n
%hold on






% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% pushbutton2 = magnitude

%---------------- to read the value -----------------------&
X=str2double(get(handles.edit1,'String')); %Amplitude
T=str2double(get(handles.edit2,'String')); %Period
H=10; %Harmonics

%---------------- set t, Vm and f -----------------------&
fs = 100000; 
t = -3*T:1/fs:3*T;
Vm=X/2;


%---------------- The equation of the signal -----------------------&
x1 = (Vm)*sawtooth(2*pi*t/T)+Vm; %Sawtooth graph formula


%---------------- plot graph at axes1 -----------------------&
plot(t,x1,'g','LineWidth',1.5,'Parent',handles.axes1);


%----------- set the location of the graph at axes1 ---------------&
axes(handles.axes1);
axis([-3*T-(0.05*T) 3*T+(0.15*T) 0 X+(0.02*X)]);


%---------------- label axis and title -----------------------&
xlabel('Time','FontSize',10);
ylabel('Amplitude','FontSize',10);
title('Sawtooth','FontSize',10);


%---------------- Integrate part -----------------------&
syms t n % let the equation in term of (t Vm pi T n)
a0 = int(((X/T)*t),t,0,T)/T; % a0 Formula
c0 = a0; %Little bit declaration

w0 = 2*pi/T; % w0 Formula
cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); % cn Formula


%set k where k equal to n. because n been used in the syms so we could not
%let n = to value
k = 1:H;

c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T); % +ve cn
c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1); % -ve cn


%------------ plot the stem graph for amplitude -------------------&
c1';
c2';
mgc1= abs(c1);
mgc2= abs(c2);
axes(handles.axes2);
stem(0,a0);
hold on
stem(-k,mgc2); % -ve n
hold on
stem(k,mgc1); % +ve n
xlabel('w (rad/s)')
ylabel('theta')
ttle = ['Phase Spectrum'];
title(ttle);
grid;
hold on

%---------------- Exponential List ---------------------&
dat(1,:)={0,abs(-X),0};

for n = 0;
        an = (X/2);
        bn = atan((X/((n)*pi))/0);               
        bn = radtodeg(n);
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


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% pushbutton3 = phase

%---------------- to read the value -----------------------&
X=str2double(get(handles.edit1,'String')); %Amplitude
T=str2double(get(handles.edit2,'String')); %Period
H=10; %Harmonics

%---------------- set t, Vm and f -----------------------&
fs = 100000; 
t = -3*T:1/fs:3*T;
Vm=X/2;


%---------------- The equation of the signal -----------------------&
x1 = (Vm)*sawtooth(2*pi*t/T)+Vm; %Sawtooth graph formula


%---------------- plot graph at axes1 -----------------------&
plot(t,x1,'g','LineWidth',1.5,'Parent',handles.axes1);


%----------- set the location of the graph at axes1 ---------------&
axes(handles.axes1);
axis([-3*T-(0.05*T) 3*T+(0.15*T) 0 X+(0.02*X)]);


%---------------- label axis and title -----------------------&
xlabel('Time','FontSize',10);
ylabel('Amplitude','FontSize',10);
title('Sawtooth','FontSize',10);


%---------------- Integrate part -----------------------&
syms t n % let the equation in term of (t Vm pi T n)
a0 = int(((X/T)*t),t,0,T)/T; % a0 Formula
c0 = a0; %Little bit declaration

w0 = 2*pi/T; % w0 Formula
cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); % cn Formula


%set k where k equal to n. because n been used in the syms so we could not
%let n = to value
k = 1:H;

c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T); % +ve cn
c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1); % -ve cn


%------------ plot the stem graph for amplitude -------------------&
c1';
c2';
mgc1= abs(c1);
mgc2= abs(c2);
axes(handles.axes2);
stem(0,a0);
hold on
stem(-k,mgc2); % -ve n
hold on
stem(k,mgc1); % +ve n

xlabel('n')
ylabel('An')
ttle = ['Amplitude Spectrum'];
title(ttle); 
grid;
hold on;
    
%---------------- set the phase angle for the graph ---------------------&
axes(handles.axes3);
zp=[c1];
p=angle(double(zp));
phasep= (p*180/pi);

zn=[c2];
pn=angle(double(zn));
phasen= (p*180/pi)*(-1);

stem(0,0);
hold on
stem(-k,phasen); %-ve n
hold on
stem(k,phasep); %+ve n
hold on

xlabel('w (rad/s)')
ylabel('theta')
ttle = ['Phase Spectrum'];
title(ttle);
grid;
hold on;


function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% edit3 = time scale

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
% popupmenu1 = gibbs phenomenon

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
% pushbutton4 = Gibbs plot

axes(handles.axes4)
N = get(handles.popupmenu1,'value'); % Gibbs Plot
X=str2double(get(handles.edit1,'String')); % amplitude
T=str2double(get(handles.edit2,'String')); % period

w0 = 2*pi/T;                          
c0 = X/2;                           
%t = -5:0.01:5;                    
fs = 100000; 
t = -3*T:1/fs:3*T;

yt = c0*ones(size(t));             
%yt = 0;
for n = 1:1:N                    
  cn = (-1/(1i*n*w0));
  yt = yt + 2*abs(cn)*cos(n*w0*t+angle(cn)); 
  %cn = (1i*X)/(2*pi*n);
  %yt = yt + 2*abs(cn)*exp(1i*n*w0*t);
  %R = R + Cn*exp(1j*nh*2*pi/T.*t); 
  %yt = real(yt);
end

plot([-3 -2 -2 -1 -1  0 0 1  1  2 2 3],...    
     [0  1 0 1 0 1 0 1 0 1 0 1], ':');
hold;                              
plot(t,yt);
grid on;
xlabel('t (seconds)'); ylabel('y(t)');
myOptions = {'1','5','15','50','99'};
ttle = strcat({'Fourier Series with N = '}, myOptions(N)); % strcat no space 
title(ttle);
hold;


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% pushbutton5 = scale

X=str2double(get(handles.edit1,'String')); % Amplitude
T=str2double(get(handles.edit2,'String')); % Period
S=str2double(get(handles.edit3,'String')); % Time Scale


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

H = 10;
w0 = 2*pi/T; 
cn = (1/n)*int(((X/T)*t)*exp(-1i*w0*t),t,0,T)*(1/T); 

%set k where k equal to n. because n been used in the syms so we could not
%let n = to value
k = 1:H;

c1 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T); % +ve cn
c2 = int(((X/T)*t)*exp(-1i*k*w0*t),t,0,T)*(1/T)*(-1); % -ve cn


%------------ plot the stem graph for amplitude -------------------&
c1';
c2';
mgc1= abs(c1);
mgc2= abs(c2);
axes(handles.axes2);
stem(0,a0);
hold on
stem(-k,mgc2); % -ve n
hold on
stem(k,mgc1); % +ve n

xlabel('n')
ylabel('An')
ttle = ['Amplitude Spectrum'];
title(ttle); 
grid;
hold on;

%---------------- set the phase angle for the graph ---------------------&
axes(handles.axes3);
zp=[c1];
p=angle(double(zp));
phasep= (p*180/pi);

zn=[c2];
pn=angle(double(zn));
phasen= (p*180/pi)*(-1);

stem(0,0);
hold on
stem(-k,phasen); %-ve n
hold on
stem(k,phasep); %+ve n
hold on


xlabel('w (rad/s)')
ylabel('theta')
ttle = ['Phase Spectrum'];
title(ttle);
grid;
hold on;


%---------------- Exponential List ---------------------&
dat(1,:)={0,abs(-X),0};

for n = 0;
        an = (X/2);
        bn = atan((X/((n)*pi))/0);               
        bn = radtodeg(n);
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


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% pushbutton8 = reset

%axes(handles.axes1);
%axes(handles.axes2); % Make averSpec the current axes.
%axes(handles.axes3);
%axes(handles.axes4);
set(handles.edit1,'String','');
set(handles.edit2,'String','');
set(handles.edit3,'String','');
arrayfun(@cla,findall(0,'type','axes'))
set(handles.uitable1, 'Data', cell(size(get(handles.uitable1,'Data'))));
cla reset; % Do a complete and total reset of the axes.
