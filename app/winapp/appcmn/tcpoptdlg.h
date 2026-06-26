//---------------------------------------------------------------------------
#ifndef tcpoptdlgH
#define tcpoptdlgH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>

#define MAXHIST		10

//---------------------------------------------------------------------------
class TTcpOptDialog : public TForm
{
__published:
	TButton *BtnCancel;
	TButton *BtnOk;
	TEdit *Port;
	TLabel *LabelAddr;
	TLabel *LabelPort;
	TEdit *User;
	TEdit *Passwd;
	TLabel *LabelUser;
	TLabel *LabelPasswd;
	TLabel *LabelMntPnt;
	TComboBox *Addr;
	TComboBox *MntPnt;
	TButton *BtnNtrip;
	TButton *BtnMountp;
	TButton *BtnBrows;
	TLabel *LabelNtripVer;
	TLabel *LabelEnableTLS;
	TLabel *LabelTLSVerify;
	TComboBox *NtripVer;
	TComboBox *EnableTLS;
	TComboBox *TLSVerify;
	void __fastcall FormShow(TObject *Sender);
	void __fastcall BtnOkClick(TObject *Sender);
	void __fastcall BtnNtripClick(TObject *Sender);
	void __fastcall BtnMountpClick(TObject *Sender);
	void __fastcall BtnBrowsClick(TObject *Sender);
	void __fastcall EnableTLSChange(TObject *Sender);
private:
        unsigned prevenabletls = 0;
	int __fastcall ExecCmd(AnsiString cmd, int show);
	void __fastcall AddHist(TComboBox *list, AnsiString *hist);
	void __fastcall UpdateEnable(void);
public:
	int Opt;
	AnsiString Path,MntpStr,History[MAXHIST];
	__fastcall TTcpOptDialog(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TTcpOptDialog *TcpOptDialog;
//---------------------------------------------------------------------------
#endif
