#ifndef AppVersion
  #define AppVersion "0.1.0"
#endif

[Setup]
AppName=OBD-II Dashboard
AppVersion={#AppVersion}
AppPublisher=Kieran Drewett
AppPublisherURL=https://github.com/kierandrewett/obd
DefaultDirName={autopf}\OBD-II Dashboard
DefaultGroupName=OBD-II Dashboard
SetupIconFile=icon.ico
Compression=lzma
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=admin

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional icons:"

[Files]
Source: "obd-dashboard.exe"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\OBD-II Dashboard"; Filename: "{app}\obd-dashboard.exe"
Name: "{commondesktop}\OBD-II Dashboard"; Filename: "{app}\obd-dashboard.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\obd-dashboard.exe"; Description: "Launch OBD-II Dashboard"; Flags: nowait postinstall skipifsilent
