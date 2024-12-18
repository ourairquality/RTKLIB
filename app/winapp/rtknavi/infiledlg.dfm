object InputFileDialog: TInputFileDialog
  Left = 0
  Top = 3
  BorderIcons = [biSystemMenu]
  BorderStyle = bsDialog
  Caption = 'Input Files'
  ClientHeight = 204
  ClientWidth = 405
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

  object LabelF1: TLabel
    Left = 8
    Top = 16
    Width = 200
    Height = 13
    Caption = 'Input File Paths (loaded at or after startup)'
  end
  object BtnKeyword: TSpeedButton
    Left = 210
    Top = 16
    Width = 25
    Height = 13
    Caption = '?'
    Flat = True
    OnClick = BtnKeywordClick
  end
  object FilePath1: TEdit
    Left = 6
    Top = 30
    Width = 366
    Height = 21
    TabOrder = 2
  end
  object BtnFile1: TButton
    Left = 373
    Top = 29
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 3
    OnClick = BtnFile1Click
  end
  object FilePath2: TEdit
    Left = 6
    Top = 52
    Width = 366
    Height = 21
    TabOrder = 4
  end
  object BtnFile2: TButton
    Left = 373
    Top = 51
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 5
    OnClick = BtnFile2Click
  end
  object FilePath3: TEdit
    Left = 6
    Top = 74
    Width = 366
    Height = 21
    TabOrder = 6
  end
  object BtnFile3: TButton
    Left = 373
    Top = 73
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 7
    OnClick = BtnFile3Click
  end
  object FilePath4: TEdit
    Left = 6
    Top = 95
    Width = 366
    Height = 21
    TabOrder = 8
  end
  object BtnFile4: TButton
    Left = 373
    Top = 94
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 9
    OnClick = BtnFile4Click
  end
  object FilePath5: TEdit
    Left = 6
    Top = 117
    Width = 366
    Height = 21
    TabOrder = 10
  end
  object BtnFile5: TButton
    Left = 373
    Top = 117
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 11
    OnClick = BtnFile5Click
  end
  object FilePath6: TEdit
    Left = 6
    Top = 139
    Width = 366
    Height = 21
    TabOrder = 12
  end
  object BtnFile6: TButton
    Left = 373
    Top = 139
    Width = 22
    Height = 22
    Caption = '...'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -9
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 13
    OnClick = BtnFile6Click
  end
  object BtnCancel: TButton
    Left = 313
    Top = 173
    Width = 85
    Height = 27
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 0
  end
  object BtnOk: TButton
    Left = 224
    Top = 173
    Width = 85
    Height = 27
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 1
    OnClick = BtnOkClick
  end
  object OpenDialog: TOpenDialog
    Filter = 
      'All File (*.*)|*.*|Precise Ephemeris (*.sp3)|*.sp3|Clock RINEX (*.clk)|' +
      '*.clk|Earth Orientation Parameters (*.erp)|*.erp'
    Options = [ofHideReadOnly, ofNoChangeDir, ofEnableSizing]
    Title = 'Input File Path'
    Left = 323
    Top = 147
  end
end
