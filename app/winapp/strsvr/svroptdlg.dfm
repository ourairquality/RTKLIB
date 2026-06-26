object SvrOptDialog: TSvrOptDialog
  Left = 0
  Top = 0
  BorderIcons = []
  BorderStyle = bsDialog
  Caption = 'Options'
  ClientHeight = 496
  ClientWidth = 435
  Color = clWhite
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  Position = poMainFormCenter
  OnShow = FormShow
  PixelsPerInch = 96
  TextHeight = 13
  object Label15: TLabel
    Left = 6
    Top = 275
    Width = 36
    Height = 13
    Caption = 'Log File'
  end
  object Label10: TLabel
    Left = 6
    Top = 254
    Width = 90
    Height = 13
    Caption = 'HTTP/NTRIP Proxy'
  end
  object Label4: TLabel
    Left = 6
    Top = 232
    Width = 90
    Height = 13
    Caption = 'FTP/HTTP Local Dir'
  end
  object Label1: TLabel
    Left = 6
    Top = 10
    Width = 90
    Height = 13
    Caption = 'Buffer Size (bytes)'
  end
  object Label2: TLabel
    Left = 6
    Top = 32
    Width = 88
    Height = 13
    Caption = 'Server Cycle  (ms)'
  end
  object Label3: TLabel
    Left = 6
    Top = 54
    Width = 104
    Height = 13
    Caption = 'Inactive Timeout (ms)'
  end
  object Label6: TLabel
    Left = 234
    Top = 10
    Width = 93
    Height = 13
    Caption = 'Period of Rate (ms)'
  end
  object Label7: TLabel
    Left = 234
    Top = 76
    Width = 82
    Height = 13
    Caption = 'Output Log Level'
  end
  object Label8: TLabel
    Left = 6
    Top = 144
    Width = 71
    Height = 13
    Caption = 'Lat/Lon/Height'
  end
  object Label9: TLabel
    Left = 234
    Top = 32
    Width = 96
    Height = 13
    Caption = 'File Swap Margin (s)'
  end
  object Label5: TLabel
    Left = 6
    Top = 76
    Width = 119
    Height = 13
    Caption = 'Reconnect Interval  (ms)'
  end
  object Label11: TLabel
    Left = 6
    Top = 166
    Width = 81
    Height = 13
    Caption = 'Offset E/N/U (m)'
  end
  object Label12: TLabel
    Left = 6
    Top = 188
    Width = 64
    Height = 13
    Caption = 'Antenna Info'
  end
  object Label13: TLabel
    Left = 6
    Top = 210
    Width = 65
    Height = 13
    Caption = 'Receiver Info'
  end
  object Label16: TLabel
    Left = 234
    Top = 54
    Width = 77
    Height = 13
    Caption = 'Relay Messages'
  end
  object Label17: TLabel
    Left = 6
    Top = 98
    Width = 118
    Height = 13
    Caption = 'Progress Bar Range (KB)'
  end
  object SvrBuffSize: TEdit
    Left = 131
    Top = 7
    Width = 85
    Height = 21
    TabOrder = 2
    Text = '16384'
  end
  object SvrCycle: TEdit
    Left = 131
    Top = 29
    Width = 85
    Height = 21
    TabOrder = 3
    Text = '100'
  end
  object DataTimeout: TEdit
    Left = 131
    Top = 51
    Width = 85
    Height = 21
    TabOrder = 4
    Text = '10000'
  end
  object ConnectInterval: TEdit
    Left = 131
    Top = 73
    Width = 85
    Height = 21
    TabOrder = 5
    Text = '2000'
  end
  object AvePeriodRate: TEdit
    Left = 340
    Top = 7
    Width = 85
    Height = 21
    TabOrder = 7
    Text = '1000'
  end
  object TraceLevelS: TComboBox
    Left = 340
    Top = 73
    Width = 85
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 10
    Text = 'None'
    Items.Strings = (
      'None'
      'Level 1'
      'Level 2'
      'Level 3'
      'Level 4'
      'Level 5')
  end
  object AntPos2: TEdit
    Left = 205
    Top = 140
    Width = 97
    Height = 21
    TabOrder = 16
    Text = '0.000'
  end
  object AntPos1: TEdit
    Left = 108
    Top = 140
    Width = 96
    Height = 21
    TabOrder = 15
    Text = '0.000'
  end
  object NmeaCycle: TEdit
    Left = 340
    Top = 95
    Width = 85
    Height = 21
    TabOrder = 12
    Text = '0'
  end
  object AntPos3: TEdit
    Left = 303
    Top = 140
    Width = 97
    Height = 21
    TabOrder = 17
    Text = '0.000'
  end
  object BtnPos: TButton
    Left = 401
    Top = 139
    Width = 25
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 18
    OnClick = BtnPosClick
  end
  object NmeaReqT: TCheckBox
    Left = 233
    Top = 97
    Width = 100
    Height = 17
    Caption = 'NMEA Cycle (ms)'
    TabOrder = 11
    OnClick = NmeaReqTClick
  end
  object LocalDir: TEdit
    Left = 125
    Top = 228
    Width = 275
    Height = 21
    TabOrder = 24
  end
  object BtnLocalDir: TButton
    Left = 401
    Top = 228
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 25
    OnClick = BtnLocalDirClick
  end
  object FileSwapMarginE: TEdit
    Left = 340
    Top = 29
    Width = 85
    Height = 21
    TabOrder = 8
    Text = '30'
  end
  object ProxyAddr: TEdit
    Left = 125
    Top = 250
    Width = 300
    Height = 21
    TabOrder = 26
  end
  object AntInfo: TEdit
    Left = 108
    Top = 184
    Width = 317
    Height = 21
    TabOrder = 22
  end
  object RcvInfo: TEdit
    Left = 108
    Top = 206
    Width = 316
    Height = 21
    TabOrder = 23
  end
  object AntOff1: TEdit
    Left = 108
    Top = 162
    Width = 96
    Height = 21
    TabOrder = 19
    Text = '0.000'
  end
  object AntOff2: TEdit
    Left = 205
    Top = 162
    Width = 97
    Height = 21
    TabOrder = 20
    Text = '0.000'
  end
  object AntOff3: TEdit
    Left = 303
    Top = 162
    Width = 97
    Height = 21
    TabOrder = 21
    Text = '0.000'
  end
  object StationId: TEdit
    Left = 131
    Top = 117
    Width = 85
    Height = 21
    TabOrder = 14
    Text = '1234'
  end
  object StaInfoSel: TCheckBox
    Left = 6
    Top = 119
    Width = 77
    Height = 17
    Caption = 'Station ID'
    TabOrder = 13
    OnClick = StaInfoSelClick
  end
  object LogFileF: TEdit
    Left = 125
    Top = 272
    Width = 250
    Height = 21
    TabOrder = 27
  end
  object BtnLogFile: TButton
    Left = 401
    Top = 272
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 28
    OnClick = BtnLogFileClick
  end
  object RelayMsg: TComboBox
    Left = 340
    Top = 51
    Width = 85
    Height = 21
    Style = csDropDownList
    TabOrder = 9
    Items.Strings = (
      'None'
      '(1)  -> (0)'
      '(2)  -> (0)'
      '(3)  -> (0)'
      '(4)  -> (0)'
      '(5)  -> (0)'
      '(6)  -> (0)')
  end
  object ProgBarR: TEdit
    Left = 131
    Top = 95
    Width = 85
    Height = 21
    TabOrder = 6
    Text = '2000'
  end
  object Label60: TLabel
    Left = 6
    Top = 296
    Width = 36
    Height = 13
    Caption = 'TLS server certificate'
  end
  object TLSSvrCertFile: TEdit
    Left = 125
    Top = 293
    Width = 275
    Height = 21
    TabOrder = 29
  end
  object BtnTLSSvrCertFile: TButton
    Left = 401
    Top = 293
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 30
    OnClick = BtnTLSSvrCertFileClick
  end
  object Label61: TLabel
    Left = 6
    Top = 317
    Width = 36
    Height = 13
    Caption = 'TLS server private key'
  end
  object TLSSvrKeyFile: TEdit
    Left = 125
    Top = 314
    Width = 275
    Height = 21
    TabOrder = 31
  end
  object BtnTLSSvrKeyFile: TButton
    Left = 401
    Top = 314
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 32
    OnClick = BtnTLSSvrKeyFileClick
  end
  object Label62: TLabel
    Left = 6
    Top = 338
    Width = 36
    Height = 13
    Caption = 'TLS server CA file'
  end
  object TLSSvrCAFile: TEdit
    Left = 125
    Top = 335
    Width = 275
    Height = 21
    TabOrder = 33
  end
  object BtnTLSSvrCAFile: TButton
    Left = 401
    Top = 335
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 34
    OnClick = BtnTLSSvrCAFileClick
  end
  object Label63: TLabel
    Left = 6
    Top = 359
    Width = 36
    Height = 13
    Caption = 'TLS server CA directory'
  end
  object TLSSvrCADir: TEdit
    Left = 125
    Top = 356
    Width = 275
    Height = 21
    TabOrder = 35
  end
  object BtnTLSSvrCADir: TButton
    Left = 401
    Top = 356
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 36
    OnClick = BtnTLSSvrCADirClick
  end
  object Label64: TLabel
    Left = 6
    Top = 380
    Width = 36
    Height = 13
    Caption = 'TLS client certificate'
  end
  object TLSCliCertFile: TEdit
    Left = 125
    Top = 377
    Width = 275
    Height = 21
    TabOrder = 37
  end
  object BtnTLSCliCertFile: TButton
    Left = 401
    Top = 377
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 38
    OnClick = BtnTLSCliCertFileClick
  end
  object Label65: TLabel
    Left = 6
    Top = 401
    Width = 36
    Height = 13
    Caption = 'TLS client private key'
  end
  object TLSCliKeyFile: TEdit
    Left = 125
    Top = 398
    Width = 275
    Height = 21
    TabOrder = 39
  end
  object BtnTLSCliKeyFile: TButton
    Left = 401
    Top = 398
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 40
    OnClick = BtnTLSCliKeyFileClick
  end
  object Label66: TLabel
    Left = 6
    Top = 422
    Width = 36
    Height = 13
    Caption = 'TLS client CA file'
  end
  object TLSCliCAFile: TEdit
    Left = 125
    Top = 419
    Width = 275
    Height = 21
    TabOrder = 41
  end
  object BtnTLSCliCAFile: TButton
    Left = 401
    Top = 419
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 42
    OnClick = BtnTLSCliCAFileClick
  end
  object Label67: TLabel
    Left = 6
    Top = 443
    Width = 36
    Height = 13
    Caption = 'TLS client CA directory'
  end
  object TLSCliCADir: TEdit
    Left = 125
    Top = 440
    Width = 275
    Height = 21
    TabOrder = 43
  end
  object BtnTLSCliCADir: TButton
    Left = 401
    Top = 440
    Width = 25
    Height = 21
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 44
    OnClick = BtnTLSCliCADirClick
  end
  object BtnOk: TButton
    Left = 233
    Top = 464
    Width = 95
    Height = 29
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtnCancel: TButton
    Left = 331
    Top = 464
    Width = 95
    Height = 29
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object OpenDialog: TOpenDialog
    Filter = 'All (*.*)|*.*|' +
      'TLS Certificate File (*.crt *.cer *.pem)|*.crt;*.cer;*.pem|' +
      'TLS Private Key File (*.key *.pem)|*.key;*.pem|'
    Left = 38
    Top = 210
  end
end
