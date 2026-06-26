//---------------------------------------------------------------------------
#include <vcl.h>
#include <FileCtrl.hpp>
#pragma hdrstop

#include "rtklib.h"
#include "serioptdlg.h"
#include "fileoptdlg.h"
#include "tcpoptdlg.h"
#include "cmdoptdlg.h"
#include "conndlg.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TConnectDialog *ConnectDialog;
//---------------------------------------------------------------------------
__fastcall TConnectDialog::TConnectDialog(TComponent* Owner)
	: TForm(Owner)
{
	Stream1=Stream2=Format1=Format2=0;
	CmdEna1[0]=CmdEna1[1]=CmdEna2[0]=CmdEna2[1]=0;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::FormShow(TObject *Sender)
{
	AnsiString s;
	int str[]={STR_NONE,STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_UDPSVR,STR_FILE};
	for (int i=0;i<7;i++) {
		if (str[i]==Stream1) SelStream1->ItemIndex=i;
		if (str[i]==Stream2) SelStream2->ItemIndex=i;
	}
	SolFormat1->ItemIndex=Format1;
	SolFormat2->ItemIndex=Format2;
	TimeFormS->ItemIndex=TimeForm;
	DegFormS ->ItemIndex=DegForm;
	FieldSepS->Text     =FieldSep;
	TimeOutTimeE->Text=s.sprintf("%d",TimeOutTime);
	ReConnTimeE ->Text=s.sprintf("%d",ReConnTime);
	TLSSvrCertFile->Text = TLSSvrCertFileF;
	TLSSvrKeyFile->Text = TLSSvrKeyFileF;
	TLSSvrCAFile->Text = TLSSvrCAFileF;
	TLSSvrCADir->Text = TLSSvrCADirectory;
	TLSCliCertFile->Text = TLSCliCertFileF;
	TLSCliKeyFile->Text = TLSCliKeyFileF;
	TLSCliCAFile->Text = TLSCliCAFileF;
	TLSCliCADir->Text = TLSCliCADirectory;
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnOkClick(TObject *Sender)
{
	int str[]={STR_NONE,STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_UDPSVR,STR_FILE};
	Stream1=str[SelStream1->ItemIndex];
	Stream2=str[SelStream2->ItemIndex];
	Format1=SolFormat1->ItemIndex;
	Format2=SolFormat2->ItemIndex;
	TimeForm=TimeFormS->ItemIndex;
	DegForm =DegFormS ->ItemIndex;
	FieldSep=FieldSepS->Text;
	TimeOutTime=TimeOutTimeE->Text.ToInt();
	ReConnTime =ReConnTimeE ->Text.ToInt();
        TLSSvrCertFileF = TLSSvrCertFile->Text;
        TLSSvrKeyFileF = TLSSvrKeyFile->Text;
        TLSSvrCAFileF = TLSSvrCAFile->Text;
        TLSSvrCADirectory = TLSSvrCADir->Text;
        TLSCliCertFileF = TLSCliCertFile->Text;
        TLSCliKeyFileF = TLSCliKeyFile->Text;
        TLSCliCAFileF = TLSCliCAFile->Text;
        TLSCliCADirectory = TLSCliCADir->Text;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnOpt1Click(TObject *Sender)
{
	switch (SelStream1->ItemIndex) {
      case 1: SerialOpt1(0, 0); break;
      case 2: TcpOpt1 (1, 1);   break;
      case 3: TcpOpt1 (0, 2);   break;
      case 4: TcpOpt1 (3, 3);   break;
      case 5: TcpOpt1 (5, 4);   break;
      case 6: TcpOpt1 (6, 5);   break;
      case 7: FileOpt1(0, 6);   break;
	}
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnOpt2Click(TObject *Sender)
{
	switch (SelStream2->ItemIndex) {
      case 1: SerialOpt2(0, 0); break;
      case 2: TcpOpt2 (1, 1);   break;
      case 3: TcpOpt2 (0, 2);   break;
      case 4: TcpOpt2 (3, 3);   break;
      case 5: TcpOpt2 (5, 4);   break;
      case 6: TcpOpt2 (6, 5);   break;
      case 7: FileOpt2(0, 6);   break;
	}
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnCmd1Click(TObject *Sender)
{
	CmdOptDialog->Cmds  [0]=Cmds1  [0];
	CmdOptDialog->Cmds  [1]=Cmds1  [1];
	CmdOptDialog->CmdEna[0]=CmdEna1[0];
	CmdOptDialog->CmdEna[1]=CmdEna1[1];
	if (CmdOptDialog->ShowModal()!=mrOk) return;
	Cmds1  [0]=CmdOptDialog->Cmds  [0];
	Cmds1  [1]=CmdOptDialog->Cmds  [1];
	CmdEna1[0]=CmdOptDialog->CmdEna[0];
	CmdEna1[1]=CmdOptDialog->CmdEna[1];
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnCmd2Click(TObject *Sender)
{
	CmdOptDialog->Cmds  [0]=Cmds2  [0];
	CmdOptDialog->Cmds  [1]=Cmds2  [1];
	CmdOptDialog->CmdEna[0]=CmdEna2[0];
	CmdOptDialog->CmdEna[1]=CmdEna2[1];
	if (CmdOptDialog->ShowModal()!=mrOk) return;
	Cmds2  [0]=CmdOptDialog->Cmds  [0];
	Cmds2  [1]=CmdOptDialog->Cmds  [1];
	CmdEna2[0]=CmdOptDialog->CmdEna[0];
	CmdEna2[1]=CmdOptDialog->CmdEna[1];
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SelStream1Change(TObject *Sender)
{
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SelStream2Change(TObject *Sender)
{
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SolFormat1Change(TObject *Sender)
{
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SolFormat2Change(TObject *Sender)
{
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SerialOpt1(int opt, unsigned i)
{
	SerialOptDialog->Path=Paths1[i];
	SerialOptDialog->Opt=opt;
	if (SerialOptDialog->ShowModal()!=mrOk) return;
	Paths1[i]=SerialOptDialog->Path;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::SerialOpt2(int opt, unsigned i)
{
	SerialOptDialog->Path=Paths2[i];
	SerialOptDialog->Opt=opt;
	if (SerialOptDialog->ShowModal()!=mrOk) return;
	Paths2[i]=SerialOptDialog->Path;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::TcpOpt1(int opt, unsigned i)
{
    // Initialize the TLS certificates.
    AnsiString SvrCertFile = TLSSvrCertFile->Text, SvrKeyFile = TLSSvrKeyFile->Text;
    AnsiString SvrCAFile = TLSSvrCAFile->Text, SvrCADir = TLSSvrCADir->Text;
    AnsiString CliCertFile = TLSCliCertFile->Text, CliKeyFile = TLSCliKeyFile->Text;
    AnsiString CliCAFile = TLSCliCAFile->Text, CliCADir = TLSCliCADir->Text;
    strinittls(SvrCertFile.c_str(), SvrKeyFile.c_str(), SvrCAFile.c_str(), SvrCADir.c_str(), 0,
               CliCertFile.c_str(), CliKeyFile.c_str(), CliCAFile.c_str(), CliCADir.c_str(), 0);
    TcpOptDialog->Path=Paths1[i];
	TcpOptDialog->Opt=opt;
	for (int i=0;i<MAXHIST;i++) TcpOptDialog->History[i]=TcpHistory[i];
	if (TcpOptDialog->ShowModal()!=mrOk) return;
	Paths1[i]=TcpOptDialog->Path;
	for (int i=0;i<MAXHIST;i++) TcpHistory[i]=TcpOptDialog->History[i];
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::TcpOpt2(int opt, unsigned i)
{
    // Initialize the TLS certificates.
    AnsiString SvrCertFile = TLSSvrCertFile->Text, SvrKeyFile = TLSSvrKeyFile->Text;
    AnsiString SvrCAFile = TLSSvrCAFile->Text, SvrCADir = TLSSvrCADir->Text;
    AnsiString CliCertFile = TLSCliCertFile->Text, CliKeyFile = TLSCliKeyFile->Text;
    AnsiString CliCAFile = TLSCliCAFile->Text, CliCADir = TLSCliCADir->Text;
    strinittls(SvrCertFile.c_str(), SvrKeyFile.c_str(), SvrCAFile.c_str(), SvrCADir.c_str(), 0,
               CliCertFile.c_str(), CliKeyFile.c_str(), CliCAFile.c_str(), CliCADir.c_str(), 0);
	TcpOptDialog->Path=Paths2[i];
	TcpOptDialog->Opt=opt;
	for (int i=0;i<MAXHIST;i++) TcpOptDialog->History[i]=TcpHistory[i];
	if (TcpOptDialog->ShowModal()!=mrOk) return;
	Paths2[i]=TcpOptDialog->Path;
	for (int i=0;i<MAXHIST;i++) TcpHistory[i]=TcpOptDialog->History[i];
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::FileOpt1(int opt, unsigned i)
{
	FileOptDialog->Path=Paths1[i];
	FileOptDialog->Opt=opt;
	if (FileOptDialog->ShowModal()!=mrOk) return;
	Paths1[i]=FileOptDialog->Path;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::FileOpt2(int opt, unsigned i)
{
	FileOptDialog->Path=Paths2[i];
	FileOptDialog->Opt=opt;
	if (FileOptDialog->ShowModal()!=mrOk) return;
	Paths2[i]=FileOptDialog->Path;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::UpdateEnable(void)
{
	BtnOpt1     ->Enabled=SelStream1->ItemIndex>0;
	BtnOpt2     ->Enabled=SelStream2->ItemIndex>0;
	BtnCmd1     ->Enabled=SelStream1->ItemIndex==1;
	BtnCmd2     ->Enabled=SelStream2->ItemIndex==1;
	SolFormat1  ->Enabled=SelStream1->ItemIndex>0;
	SolFormat2  ->Enabled=SelStream2->ItemIndex>0;
	TimeFormS   ->Enabled=SolFormat1->ItemIndex!=3||SolFormat2->ItemIndex!=3;
	DegFormS    ->Enabled=SolFormat1->ItemIndex==0||SolFormat2->ItemIndex==0;
	FieldSepS   ->Enabled=SolFormat1->ItemIndex!=3||SolFormat2->ItemIndex!=3;
	Label5      ->Enabled=SolFormat1->ItemIndex!=3||SolFormat2->ItemIndex!=3;
	Label6      ->Enabled=SolFormat1->ItemIndex==0||SolFormat2->ItemIndex==0;
	Label7      ->Enabled=SolFormat1->ItemIndex!=3||SolFormat2->ItemIndex!=3;
	Label8      ->Enabled=(SelStream1->ItemIndex == 2 || SelStream1->ItemIndex == 4) ||
						  (SelStream2->ItemIndex == 2 || SelStream2->ItemIndex == 4);
 	TimeOutTimeE->Enabled=(SelStream1->ItemIndex == 2 || SelStream1->ItemIndex == 4) ||
            (SelStream2->ItemIndex == 2 || SelStream2->ItemIndex == 4);
	ReConnTimeE ->Enabled=(SelStream1->ItemIndex == 2 || SelStream1->ItemIndex == 4) ||
						  (SelStream2->ItemIndex == 2 || SelStream2->ItemIndex == 4);
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSSvrCertFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Server Certificate File";
  OpenDialog->FilterIndex = 2;
  if (!OpenDialog->Execute()) return;
  TLSSvrCertFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSSvrKeyFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Server Private Key File";
  OpenDialog->FilterIndex = 3;
  if (!OpenDialog->Execute()) return;
  TLSSvrKeyFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSSvrCAFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Server CA File";
  OpenDialog->FilterIndex = 2;
  if (!OpenDialog->Execute()) return;
  TLSSvrCAFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSSvrCADirClick(TObject *Sender) {
  UnicodeString dir = TLSSvrCADir->Text;
  TSelectDirExtOpts opt = TSelectDirExtOpts() << sdNewUI << sdNewFolder;
  if (!SelectDirectory(L"TLS Server CA Directory", L"", dir, opt)) return;
  TLSSvrCADir->Text = dir;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSCliCertFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Client Certificate File";
  OpenDialog->FilterIndex = 2;
  if (!OpenDialog->Execute()) return;
  TLSCliCertFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSCliKeyFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Client Private Key File";
  OpenDialog->FilterIndex = 3;
  if (!OpenDialog->Execute()) return;
  TLSCliKeyFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSCliCAFileClick(TObject *Sender) {
  OpenDialog->Title = "TLS Client CA File";
  OpenDialog->FilterIndex = 2;
  if (!OpenDialog->Execute()) return;
  TLSCliCAFile->Text = OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TConnectDialog::BtnTLSCliCADirClick(TObject *Sender) {
  UnicodeString dir = TLSCliCADir->Text;
  TSelectDirExtOpts opt = TSelectDirExtOpts() << sdNewUI << sdNewFolder;
  if (!SelectDirectory(L"TLS Client CA Directory", L"", dir, opt)) return;
  TLSCliCADir->Text = dir;
}
//---------------------------------------------------------------------------

