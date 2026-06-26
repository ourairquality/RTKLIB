//---------------------------------------------------------------------------
#ifndef svroptdlgH
#define svroptdlgH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <Dialogs.hpp>
//---------------------------------------------------------------------------
class TSvrOptDialog : public TForm
{
__published:
	TButton *BtnOk;
	TButton *BtnCancel;
	TEdit *SvrBuffSize;
	TLabel *Label1;
	TLabel *Label2;
	TEdit *SvrCycle;
	TLabel *Label3;
	TEdit *DataTimeout;
	TLabel *Label6;
	TEdit *ConnectInterval;
	TEdit *AvePeriodRate;
	TLabel *Label7;
	TComboBox *TraceLevelS;
	TEdit *AntPos2;
	TEdit *AntPos1;
	TEdit *NmeaCycle;
	TLabel *Label8;
	TEdit *AntPos3;
	TButton *BtnPos;
	TCheckBox *NmeaReqT;
	TEdit *LocalDir;
	TButton *BtnLocalDir;
	TLabel *Label4;
	TLabel *Label9;
	TEdit *FileSwapMarginE;
	TLabel *Label5;
	TLabel *Label10;
	TEdit *ProxyAddr;
	TEdit *AntInfo;
	TEdit *RcvInfo;
	TLabel *Label11;
	TEdit *AntOff1;
	TEdit *AntOff2;
	TEdit *AntOff3;
	TLabel *Label12;
	TLabel *Label13;
	TEdit *StationId;
	TCheckBox *StaInfoSel;
	TLabel *Label15;
	TEdit *LogFileF;
	TButton *BtnLogFile;
	TOpenDialog *OpenDialog;
	TLabel *Label16;
	TComboBox *RelayMsg;
	TEdit *ProgBarR;
	TLabel *Label17;

	TLabel *Label60;
	TLabel *Label61;
	TLabel *Label62;
	TLabel *Label63;
	TLabel *Label64;
	TLabel *Label65;
	TLabel *Label66;
	TLabel *Label67;
    TEdit *TLSSvrCertFile;
    TEdit *TLSSvrKeyFile;
    TEdit *TLSSvrCAFile;
    TEdit *TLSSvrCADir;
    TEdit *TLSCliCertFile;
    TEdit *TLSCliKeyFile;
    TEdit *TLSCliCAFile;
    TEdit *TLSCliCADir;
    TButton *BtnTLSSvrCertFile;
    TButton *BtnTLSSvrKeyFile;
    TButton *BtnTLSSvrCAFile;
    TButton *BtnTLSSvrCADir;
    TButton *BtnTLSCliCertFile;
    TButton *BtnTLSCliKeyFile;
    TButton *BtnTLSCliCAFile;
    TButton *BtnTLSCliCADir;
    void __fastcall BtnTLSSvrCertFileClick(TObject *Sender);
    void __fastcall BtnTLSSvrKeyFileClick(TObject *Sender);
    void __fastcall BtnTLSSvrCAFileClick(TObject *Sender);
    void __fastcall BtnTLSSvrCADirClick(TObject *Sender);
    void __fastcall BtnTLSCliCertFileClick(TObject *Sender);
    void __fastcall BtnTLSCliKeyFileClick(TObject *Sender);
    void __fastcall BtnTLSCliCAFileClick(TObject *Sender);
    void __fastcall BtnTLSCliCADirClick(TObject *Sender);

	void __fastcall BtnOkClick(TObject *Sender);
	void __fastcall FormShow(TObject *Sender);
	void __fastcall BtnPosClick(TObject *Sender);
	void __fastcall NmeaReqTClick(TObject *Sender);
	void __fastcall BtnLocalDirClick(TObject *Sender);
	void __fastcall StaInfoSelClick(TObject *Sender);
	void __fastcall BtnLogFileClick(TObject *Sender);
private:
	void __fastcall UpdateEnable(void);
public:
	AnsiString StaPosFile,ExeDirectory,LocalDirectory,ProxyAddress;
	AnsiString AntType,RcvType,LogFile;
	int SvrOpt[6],TraceLevel,NmeaReq,FileSwapMargin,StaId,StaSel,RelayBack;
	int ProgBarRange;
	double AntPos[3],AntOff[3];
  AnsiString TLSSvrCertFileF, TLSSvrKeyFileF, TLSSvrCAFileF, TLSSvrCADirectory;
  AnsiString TLSCliCertFileF, TLSCliKeyFileF, TLSCliCAFileF, TLSCliCADirectory;
	__fastcall TSvrOptDialog(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TSvrOptDialog *SvrOptDialog;
//---------------------------------------------------------------------------
#endif
