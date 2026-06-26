//---------------------------------------------------------------------------
#include <vcl.h>
#include <stdio.h>
#pragma hdrstop

#include "rtklib.h"
#include "tcpoptdlg.h"
#include "mntpoptdlg.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TTcpOptDialog *TcpOptDialog;

#define NTRIP_TIMEOUT	5000				// response timeout (ms)
#define NTRIP_CYCLE		50					// processing cycle (ms)
#define MAXSRCTBL		512000				// max source table size (bytes)
#define ENDSRCTBL		"ENDSOURCETABLE"	// end marker of table
#define MAXLINE			1024				// max line size (byte)

//---------------------------------------------------------------------------
__fastcall TTcpOptDialog::TTcpOptDialog(TComponent* Owner)
	: TForm(Owner)
{
  prevenabletls = 0;
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::UpdateEnable(void)
{
  unsigned idx = EnableTLS->ItemIndex;
  LabelTLSVerify->Enabled = Opt >= 0 && Opt <= 5 && idx > 0;
  TLSVerify->Enabled = Opt >= 0 && Opt <= 5 && idx > 0;
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::EnableTLSChange(TObject *Sender)
{
  if (EnableTLS->ItemIndex > 0 && prevenabletls == 0) {
    // TLS is being changed to enabled.
    //
    // Default the Ntrip client version to 2 and the caster version to 0 (both).
    if (Opt == 2 || Opt == 3) NtripVer->ItemIndex = 2;
    else if (Opt == 4 || Opt == 5) NtripVer->ItemIndex = 0;
    // For a client default TLS Verify on.
    if (Opt == 1 || Opt == 2 || Opt == 3) TLSVerify->ItemIndex = 1;
    // If the port is undefined then set to 443.
	AnsiString Port_Text = Port->Text;
    if (Port_Text == "" || Port_Text == "0") Port->Text = "443";
    else if (Opt >= 2 && Opt <= 5) {
      // If this a Ntrip connection and the port is 2201 then change to 443.
      if (Port_Text == "2201") Port->Text = "443";
    }
  }
  prevenabletls = EnableTLS->ItemIndex;
  UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::FormShow(TObject *Sender)
{
	char buff[2048],*p,*q;
	char *port=(char *)"",*mntpnt=(char *)"",*user=(char *)"";
	char *passwd=(char *)"",*str=(char *)"";
	const char *ti[]={"TCP Server Options ","TCP Client Options",
					  "NTRIP Source Options","NTRIP Client Options",
					  "NTRIP Caster Options",
					  "NTRIP Caster Source Options", "UDP Server Options",
					  "UDP Client Options"};

	strcpy(buff,Path.c_str());

    // Options.
    // Default the NTRIP version to 1 for clients.
    unsigned tls = 0, tlsverify = 0, ntripversion = 0;
    const char *dc = "::";
    size_t dclen = strlen(dc);
    p = strstr(buff, dc);
    if (p != NULL) {
      *p = '\0';
      p += 2;
      q = p;
      do {
        p = strstr(q, dc);
        if (p != NULL) {
          *p = '\0';
          p += 2;
        }
        if (Opt <= 5) {
          if (q[0] == 'S') tls = 1;
          else if (q[0] == 'A') tls = (Opt != 4 && Opt != 5) ? 1 : 2;
          else if (sscanf(q, "V=%u", &tlsverify) == 1) {
            if (tlsverify > 1) tlsverify = 1;
          } else if ((Opt >= 2 && Opt <= 5) && sscanf(q, "N=%u", &ntripversion) == 1) {
            if (ntripversion > 2) ntripversion = 2;
          }
        }
        q = p;
      } while (q != NULL);
    }

	if (!(p=strchr(buff,'@'))) p=buff;
	
	if ((p=strchr(p,'/')) != NULL) {
      if (Opt >= 2 && Opt <= 5) {
        if ((q=strchr(p+1,':'))) {
          *q='\0'; str=q+1;
        }
      }
      *p='\0'; mntpnt=p+1;
	}
	if ((p=strrchr(buff,'@'))) {
		*p++='\0';
		if ((q=strchr(buff,':'))) {
			*q='\0'; passwd=q+1;
		}
		user=buff;
	}
	else p=buff;
	
	if ((q=strchr(p,':'))) {
		*q='\0'; port=q+1;
	}
	Caption=ti[Opt];
	LabelAddr  ->Caption=(Opt>=2&&Opt<=5)?"NTRIP Caster Address":"Server Address";
	LabelMntPnt->Enabled=(Opt>=2&&Opt<=5);
	MntPnt     ->Enabled=(Opt>=2&&Opt<=5);
	LabelUser  ->Enabled=(Opt>=3&&Opt<=5);
	User       ->Enabled=(Opt>=3&&Opt<=5);
	LabelPasswd->Enabled=(Opt>=2&&Opt<=5);
	Passwd     ->Enabled=(Opt>=2&&Opt<=5);
	BtnNtrip   ->Visible=(Opt==3);
	BtnBrows   ->Visible=(Opt==3);
	BtnMountp  ->Visible=(Opt==2||Opt==5);

	LabelNtripVer->Enabled = (Opt >= 2 && Opt <= 5);
	NtripVer->Enabled = (Opt >= 2 && Opt <= 5);
	NtripVer->Items->Clear();
    if (Opt == 4 || Opt == 5) NtripVer->Items->Add("Both 1 and 2");
    else NtripVer->Items->Add("Default");
    NtripVer->Items->Add("Only 1");
    NtripVer->Items->Add("Only 2");

	LabelEnableTLS->Enabled = (Opt >= 0 && Opt <= 5);
	EnableTLS->Enabled = (Opt >=0 && Opt <= 5);
    EnableTLS->Items->Clear();
    EnableTLS->Items->Add("None");
    EnableTLS->Items->Add("Require TLS");
    if (Opt >= 0 && Opt <= 5) {
      if (Opt == 4 || Opt == 5) {
        EnableTLS->Items->Add("Allow TLS");
      } else {
        if (tls > 1) tls = 1;
      }
    }
	LabelTLSVerify->Enabled = (Opt >=0 && Opt <= 5);
	TLSVerify->Enabled = (Opt >= 0 && Opt <= 5);

    if (strcmp(port, "443") == 0 && tls == 0) {
      // If port is 443 then force on TLS.
      if (Opt == 4 || Opt == 5) tls = 2; else tls = 1;
      // Default the Ntrip client version to 2 and the caster version to 0 (both).
      if (Opt == 2 || Opt == 3) ntripversion = 2;
      else if (Opt == 4 || Opt == 5) ntripversion = 0;
      // For a client default TLS Verify on.
      if (Opt == 1 || Opt == 2 || Opt == 3) tlsverify = 1;
    }


    // Unescape components.
    char addru[256], portu[256], mntpntu[256], useru[256], passwdu[256], mntpstru[256];
    strurlunescape(p, 0, SIZE_MAX, addru, sizeof(addru));
    strurlunescape(port, 0, SIZE_MAX, portu, sizeof(portu));
    strurlunescape(mntpnt, 0, SIZE_MAX, mntpntu, sizeof(mntpntu));
    strurlunescape(user, 0, SIZE_MAX, useru, sizeof(useru));
    strurlunescape(passwd, 0, SIZE_MAX, passwdu, sizeof(passwdu));
    strurlunescape(str, 0, SIZE_MAX, mntpstru, sizeof(mntpstru));

	Addr  ->Text=addru;
	Port  ->Text=portu;
    NtripVer->ItemIndex = ntripversion;
    EnableTLS->ItemIndex = tls;
    TLSVerify->ItemIndex = tlsverify ? 1 : 0;
	MntPnt->Text=mntpntu;
	User  ->Text=useru;
	Passwd->Text=passwdu;
	if (Opt==2||Opt==4) {
	    MntpStr=mntpstru;
	}
	Addr->Items->Clear();
	for (int i=0;i<MAXHIST;i++) {
		if (History[i]!="") Addr->Items->Add(History[i]);
	}
    prevenabletls = tls;
    UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::BtnOkClick(TObject *Sender)
{
	AnsiString User_Text=User->Text,Passwd_Text=Passwd->Text;
	AnsiString Addr_Text=Addr->Text,Port_Text=Port->Text;
	AnsiString MntPnt_Text=MntPnt->Text,s;

    // Escape components.
    char addr[256 * 3], port[256 * 3], mntpnt[256 * 3], user[256 * 3];
    char passwd[256 * 3], mntpstr[256 * 3];
    strurlescape(Addr_Text.c_str(), 0, SIZE_MAX, addr, sizeof(addr));
    strurlescape(Port_Text.c_str(), 0, SIZE_MAX, port, sizeof(port));
    strurlescape(MntPnt_Text.c_str(), 0, SIZE_MAX, mntpnt, sizeof(mntpnt));
    strurlescape(User_Text.c_str(), 0, SIZE_MAX, user, sizeof(user));
    strurlescape(Passwd_Text.c_str(), 0, SIZE_MAX, passwd, sizeof(passwd));
    strurlescape(MntpStr.c_str(), 0, SIZE_MAX, mntpstr, sizeof(mntpstr));
    
	Path=s.sprintf("%s:%s@%s:%s/%s",user,passwd,addr,port,mntpnt);
    if (MntpStr != "") Path=s.sprintf("%s:%s", Path.c_str(), mntpstr);

    if (NtripVer->ItemIndex > 0)
      Path = s.sprintf("%s::N=%d", Path.c_str(), NtripVer->ItemIndex);
    if (Opt <= 3) {
      unsigned idx = EnableTLS->ItemIndex;
      if (idx > 0) Path = Path + "::S";
    } else if (Opt == 4 || Opt == 5) {
      unsigned idx = EnableTLS->ItemIndex;
      if (idx == 1) Path = Path + "::S";
      else if (idx == 2) Path = Path + "::A";
    }
    if (Opt <= 5) {
      unsigned idx = TLSVerify->ItemIndex;
      Path = s.sprintf("%s::V=%d", Path.c_str(), idx > 0 ? 1 : 0) ;
    }
	AddHist(Addr,History);
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::AddHist(TComboBox *list, AnsiString *hist)
{
	for (int i=0;i<MAXHIST;i++) {
		if (list->Text!=hist[i]) continue;
		for (int j=i+1;j<MAXHIST;j++) hist[j-1]=hist[j];
		hist[MAXHIST-1]="";
	}
	for (int i=MAXHIST-1;i>0;i--) hist[i]=hist[i-1];
	hist[0]=list->Text;
	
	list->Clear();
	for (int i=0;i<MAXHIST;i++) {
		if (hist[i]!="") list->Items->Add(hist[i]);
	}
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::BtnNtripClick(TObject *Sender)
{
	TButton *btn=(TButton *)Sender;
	stream_t str;
	uint32_t tick=tickget();
	char msg[MAXSTRMSG],mntpnt[256];

    // Escape components.
	AnsiString Addr_Text=Addr->Text,Port_Text=Port->Text;
    char addr[256 * 3], port[256 * 3];
    strurlescape(Addr_Text.c_str(), 0, SIZE_MAX, addr, sizeof(addr));
    strurlescape(Port_Text.c_str(), 0, SIZE_MAX, port, sizeof(port));
    AnsiString s, path=s.sprintf("%s:%s",addr,port);
    if (NtripVer->ItemIndex > 0)
      path = s.sprintf("%s::N=%d", path.c_str(), NtripVer->ItemIndex);
    if (Opt == 4) path = path + "::T=1";
    else if (Opt == 5) path = path + "::T=2";
    if (EnableTLS->ItemIndex > 0) path = path + "::S";
    path = s.sprintf("%s::V=%d", path.c_str(), TLSVerify->ItemIndex > 0 ? 1 : 0) ;

	strinit(&str);
	if (!stropen(&str,STR_NTRIPCLI,STR_MODE_R,path.c_str())) return;
	
    char *buff = (char *)malloc(MAXSRCTBL);
    if (buff == NULL) {
      strclose(&str);
      return;
    }

	btn->Enabled=false;
    buff[0] = '\0';
    size_t pi = 0;
    while (pi + 1 < MAXSRCTBL) {
        pi += strread(&str, (uint8_t *)buff, MAXSRCTBL, pi, MAXSRCTBL - pi - 1);
        buff[pi] = '\0';
        sleepms(NTRIP_CYCLE);
        if (strstr(buff, ENDSRCTBL)) break;
        if ((int)(tickget() - tick) > NTRIP_TIMEOUT) break;
        if (strstat(&str, NULL, 0) <= 0) break;
    }
    strclose(&str);
	
	MntPnt->Clear();
	for (char *p=buff;(p=strstr(p,"STR;"));p+=4) {
		if (sscanf(p,"STR;%255[^;]",mntpnt)==1) {
			MntPnt->AddItem(mntpnt,NULL);
		}
	}
    free(buff);
	btn->Enabled=true;
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::BtnBrowsClick(TObject *Sender)
{
    AnsiString Addr_Text=Addr->Text;
    AnsiString Port_Text=Port->Text;
	if (Port_Text!="") Addr_Text+=":"+Port_Text;
    ExecCmd("srctblbrows "+Addr_Text,1);
}
//---------------------------------------------------------------------------
void __fastcall TTcpOptDialog::BtnMountpClick(TObject *Sender)
{
	MntpOptDialog->MntPnt=MntPnt->Text;
	MntpOptDialog->MntpStr=MntpStr;
    if (MntpOptDialog->ShowModal()!=mrOk) return;
	MntPnt->Text=MntpOptDialog->MntPnt;
	MntpStr=MntpOptDialog->MntpStr;
}
//---------------------------------------------------------------------------
int __fastcall TTcpOptDialog::ExecCmd(AnsiString cmd, int show)
{
    PROCESS_INFORMATION info;
    STARTUPINFO si={0};
    si.cb=sizeof(si);
    char *p=cmd.c_str();
    
    if (!CreateProcess(NULL,p,NULL,NULL,false,show?0:CREATE_NO_WINDOW,NULL,
                       NULL,&si,&info)) return 0;
    CloseHandle(info.hProcess);
    CloseHandle(info.hThread);
    return 1;
}
//---------------------------------------------------------------------------
