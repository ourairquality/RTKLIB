object ConnectDialog: TConnectDialog
  Left = 0
  Top = 0
  BorderStyle = bsDialog
  Caption = 'Connection Settings'
  ClientHeight = 337
  ClientWidth = 331
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
  object Label1: TLabel
    Left = 62
    Top = 6
    Width = 61
    Height = 13
    Caption = 'Stream Type'
  end
  object Label2: TLabel
    Left = 159
    Top = 6
    Width = 18
    Height = 13
    Caption = 'Opt'
  end
  object Label4: TLabel
    Left = 188
    Top = 6
    Width = 21
    Height = 13
    Caption = 'Cmd'
  end
  object Label3: TLabel
    Left = 230
    Top = 6
    Width = 75
    Height = 13
    Caption = 'Solution Format'
  end
  object Label9: TLabel
    Left = 16
    Top = 26
    Width = 14
    Height = 13
    Caption = '(1)'
  end
  object SelStream1: TComboBox
    Left = 42
    Top = 22
    Width = 111
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 2
    OnChange = SelStream1Change
    Items.Strings = (
      ''
      'Serial'
      'TCP Client'
      'TCP Server'
      'NTRIP Client'
      'NTRIP Caster Source'
      'UDP Server'
      'File')
  end
  object BtnOpt1: TButton
    Left = 157
    Top = 21
    Width = 25
    Height = 23
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 3
    OnClick = BtnOpt1Click
  end
  object BtnCmd1: TButton
    Left = 186
    Top = 21
    Width = 25
    Height = 23
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 4
    OnClick = BtnCmd1Click
  end
  object SolFormat1: TComboBox
    Left = 215
    Top = 22
    Width = 110
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 5
    Text = 'Lat/Lon/Height'
    OnChange = SolFormat1Change
    Items.Strings = (
      'Lat/Lon/Height'
      'X/Y/Z-ECEF'
      'E/N/U-Baseline'
      'NMEA0183'
      'Solution Status')
  end
  object Label10: TLabel
    Left = 16
    Top = 50
    Width = 14
    Height = 13
    Caption = '(2)'
  end
  object SelStream2: TComboBox
    Left = 42
    Top = 46
    Width = 111
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 6
    OnChange = SelStream2Change
    Items.Strings = (
      ''
      'Serial'
      'TCP Client'
      'TCP Server'
      'NTRIP Client'
      'NTRIP Caster Source'
      'UDP Server'
      'File')
  end
  object BtnOpt2: TButton
    Left = 157
    Top = 45
    Width = 25
    Height = 23
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 7
    OnClick = BtnOpt2Click
  end
  object BtnCmd2: TButton
    Left = 186
    Top = 45
    Width = 25
    Height = 23
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 8
    OnClick = BtnCmd2Click
  end
  object SolFormat2: TComboBox
    Left = 215
    Top = 46
    Width = 110
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 9
    Text = 'Lat/Lon/Height'
    OnChange = SolFormat2Change
    Items.Strings = (
      'Lat/Lon/Height'
      'X/Y/Z-ECEF'
      'E/N/U-Baseline'
      'NMEA0183'
      'Solution Status')
  end
  object Label5: TLabel
    Left = 28
    Top = 72
    Width = 59
    Height = 13
    Caption = 'Time Format'
  end
  object Label6: TLabel
    Left = 152
    Top = 72
    Width = 73
    Height = 13
    Caption = 'Lat/Lon Format'
  end
  object Label7: TLabel
    Left = 267
    Top = 72
    Width = 43
    Height = 13
    Caption = 'Field Sep'
  end
  object TimeFormS: TComboBox
    Left = 8
    Top = 86
    Width = 115
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 10
    Text = 'ww ssss.ss GPST'
    OnChange = SelStream1Change
    Items.Strings = (
      'ww ssss.ss GPST'
      'hh:mm:ss GPST'
      'hh:mm:ss UTC'
      'hh:mm:ss JST')
  end
  object DegFormS: TComboBox
    Left = 140
    Top = 86
    Width = 104
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 11
    Text = 'ddd.ddddddd'
    OnChange = SelStream1Change
    Items.Strings = (
      'ddd.ddddddd'
      'ddd mm ss.sss')
  end
  object FieldSepS: TEdit
    Left = 261
    Top = 86
    Width = 64
    Height = 21
    TabOrder = 12
  end
  object Label8: TLabel
    Left = 6
    Top = 114
    Width = 153
    Height = 13
    Caption = 'Timeout/Reconnect Interval (ms)'
  end
  object TimeOutTimeE: TEdit
    Left = 181
    Top = 110
    Width = 63
    Height = 21
    TabOrder = 13
    Text = '0'
  end
  object ReConnTimeE: TEdit
    Left = 261
    Top = 110
    Width = 64
    Height = 21
    TabOrder = 14
    Text = '10000'
  end
  object Label60: TLabel
    Left = 6
    Top = 134
    Width = 36
    Height = 13
    Caption = 'TLS server certificate'
  end
  object TLSSvrCertFile: TEdit
    Left = 125
    Top = 131
    Width = 173
    Height = 21
    TabOrder = 29
  end
  object BtnTLSSvrCertFile: TButton
    Left = 300
    Top = 131
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
    Top = 155
    Width = 36
    Height = 13
    Caption = 'TLS server private key'
  end
  object TLSSvrKeyFile: TEdit
    Left = 125
    Top = 152
    Width = 173
    Height = 21
    TabOrder = 31
  end
  object BtnTLSSvrKeyFile: TButton
    Left = 300
    Top = 152
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
    Top = 176
    Width = 36
    Height = 13
    Caption = 'TLS server CA file'
  end
  object TLSSvrCAFile: TEdit
    Left = 125
    Top = 173
    Width = 173
    Height = 21
    TabOrder = 33
  end
  object BtnTLSSvrCAFile: TButton
    Left = 300
    Top = 173
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
    Top = 197
    Width = 36
    Height = 13
    Caption = 'TLS server CA directory'
  end
  object TLSSvrCADir: TEdit
    Left = 125
    Top = 194
    Width = 173
    Height = 21
    TabOrder = 35
  end
  object BtnTLSSvrCADir: TButton
    Left = 300
    Top = 194
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
    Top = 218
    Width = 36
    Height = 13
    Caption = 'TLS client certificate'
  end
  object TLSCliCertFile: TEdit
    Left = 125
    Top = 215
    Width = 173
    Height = 21
    TabOrder = 37
  end
  object BtnTLSCliCertFile: TButton
    Left = 300
    Top = 215
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
    Top = 239
    Width = 36
    Height = 13
    Caption = 'TLS client private key'
  end
  object TLSCliKeyFile: TEdit
    Left = 125
    Top = 236
    Width = 173
    Height = 21
    TabOrder = 39
  end
  object BtnTLSCliKeyFile: TButton
    Left = 300
    Top = 236
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
    Top = 260
    Width = 36
    Height = 13
    Caption = 'TLS client CA file'
  end
  object TLSCliCAFile: TEdit
    Left = 125
    Top = 257
    Width = 173
    Height = 21
    TabOrder = 41
  end
  object BtnTLSCliCAFile: TButton
    Left = 300
    Top = 257
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
    Top = 281
    Width = 36
    Height = 13
    Caption = 'TLS client CA directory'
  end
  object TLSCliCADir: TEdit
    Left = 125
    Top = 278
    Width = 173
    Height = 21
    TabOrder = 43
  end
  object BtnTLSCliCADir: TButton
    Left = 300
    Top = 278
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
    Left = 141
    Top = 302
    Width = 89
    Height = 29
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtnCancel: TButton
    Left = 236
    Top = 302
    Width = 89
    Height = 29
    Cancel = True
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object OpenDialog: TOpenDialog
    Filter = 'All (*.*)|*.*' +
      '|TLS Certificate File (*.crt *.cer *.pem)|*.crt;*.cer;*.pem|' +
      'TLS Private Key File (*.key *.pem)|*.key;*.pem|'
    Left = 38
    Top = 210
  end
end
