//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop

#include "rtklib.h"
#include "convopt.h"
#include "codeopt.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TCodeOptDialog *CodeOptDialog;
//---------------------------------------------------------------------------
__fastcall TCodeOptDialog::TCodeOptDialog(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TCodeOptDialog::FormShow(TObject *Sender)
{
	char mask[7][MAXCODE+1]={""};
	
	for (int i=0;i<7;i++) strcpy(mask[i],ConvOptDialog->CodeMask[i].c_str());
	G01->Checked=mask[0][ 0]=='1';
	G02->Checked=mask[0][ 1]=='1';
	G03->Checked=mask[0][ 2]=='1';
	G04->Checked=mask[0][ 3]=='1';
	G05->Checked=mask[0][ 4]=='1';
	G06->Checked=mask[0][ 5]=='1';
	G07->Checked=mask[0][ 6]=='1';
	G08->Checked=mask[0][ 7]=='1';
	G12->Checked=mask[0][11]=='1';
	G14->Checked=mask[0][13]=='1';
	G15->Checked=mask[0][14]=='1';
	G16->Checked=mask[0][15]=='1';
	G17->Checked=mask[0][16]=='1';
	G18->Checked=mask[0][17]=='1';
	G19->Checked=mask[0][18]=='1';
	G20->Checked=mask[0][19]=='1';
	G21->Checked=mask[0][20]=='1';
	G22->Checked=mask[0][21]=='1';
	G23->Checked=mask[0][22]=='1';
	G24->Checked=mask[0][23]=='1';
	G25->Checked=mask[0][24]=='1';
	G26->Checked=mask[0][25]=='1';
	R01->Checked=mask[1][ 0]=='1';
	R02->Checked=mask[1][ 1]=='1';
	R14->Checked=mask[1][13]=='1';
	R19->Checked=mask[1][18]=='1';
	R44->Checked=mask[1][43]=='1';
	R45->Checked=mask[1][44]=='1';
	R46->Checked=mask[1][45]=='1';
	R66->Checked=mask[1][65]=='1'; //
	R67->Checked=mask[1][66]=='1'; //
	R68->Checked=mask[1][67]=='1'; //
	R30->Checked=mask[1][29]=='1'; //
	R31->Checked=mask[1][30]=='1'; //
	R33->Checked=mask[1][32]=='1'; //
	E01->Checked=mask[2][ 0]=='1';
	E10->Checked=mask[2][ 9]=='1';
	E11->Checked=mask[2][10]=='1';
	E12->Checked=mask[2][11]=='1';
	E13->Checked=mask[2][12]=='1';
	E24->Checked=mask[2][23]=='1';
	E25->Checked=mask[2][24]=='1';
	E26->Checked=mask[2][25]=='1';
	E27->Checked=mask[2][26]=='1';
	E28->Checked=mask[2][27]=='1';
	E29->Checked=mask[2][28]=='1';
	E30->Checked=mask[2][29]=='1';
	E31->Checked=mask[2][30]=='1';
	E32->Checked=mask[2][31]=='1';
	E33->Checked=mask[2][32]=='1';
	E34->Checked=mask[2][33]=='1';
	E37->Checked=mask[2][36]=='1';
	E38->Checked=mask[2][37]=='1';
	E39->Checked=mask[2][38]=='1';
	J01->Checked=mask[3][ 0]=='1';
	J07->Checked=mask[3][ 6]=='1';
	J08->Checked=mask[3][ 7]=='1';
	J09->Checked=mask[3][ 8]=='1';
	J11->Checked=mask[3][10]=='1';
	J12->Checked=mask[3][11]=='1';
	J13->Checked=mask[3][12]=='1';
	J16->Checked=mask[3][15]=='1';
	J17->Checked=mask[3][16]=='1';
	J18->Checked=mask[3][17]=='1';
	J24->Checked=mask[3][23]=='1';
	J25->Checked=mask[3][24]=='1';
	J26->Checked=mask[3][25]=='1';
	J57->Checked=mask[3][56]=='1'; //
	J58->Checked=mask[3][57]=='1'; //
	J59->Checked=mask[3][58]=='1'; //
	J60->Checked=mask[3][59]=='1'; //
	J34->Checked=mask[3][33]=='1'; //
	J35->Checked=mask[3][34]=='1';
	J36->Checked=mask[3][35]=='1';
	J33->Checked=mask[3][32]=='1';
	C40->Checked=mask[5][39]=='1'; //
	C41->Checked=mask[5][40]=='1'; //
	C18->Checked=mask[5][17]=='1'; //
	C27->Checked=mask[5][26]=='1';
	C28->Checked=mask[5][27]=='1';
	C29->Checked=mask[5][28]=='1';
	C42->Checked=mask[5][41]=='1';
	C43->Checked=mask[5][42]=='1';
	C33->Checked=mask[5][32]=='1';
	C56->Checked=mask[5][55]=='1'; //
	C02->Checked=mask[5][ 1]=='1'; //
	C12->Checked=mask[5][11]=='1'; //
	C07->Checked=mask[5][ 6]=='1'; //
	C08->Checked=mask[5][ 7]=='1'; //
	C13->Checked=mask[5][12]=='1'; //
	C57->Checked=mask[5][56]=='1'; //
	C58->Checked=mask[5][57]=='1'; //
	C26->Checked=mask[5][25]=='1'; //
	C61->Checked=mask[5][60]=='1'; //
	C62->Checked=mask[5][61]=='1'; //
	C63->Checked=mask[5][62]=='1'; //
	C64->Checked=mask[5][63]=='1'; //
	C65->Checked=mask[5][64]=='1'; //
	C39->Checked=mask[5][38]=='1'; //
	C69->Checked=mask[5][68]=='1'; //
	C70->Checked=mask[5][69]=='1'; //
	C34->Checked=mask[5][33]=='1'; //
	I49->Checked=mask[6][48]=='1';
	I50->Checked=mask[6][49]=='1';
	I51->Checked=mask[6][50]=='1';
	I26->Checked=mask[6][25]=='1';
	I52->Checked=mask[6][51]=='1';
	I53->Checked=mask[6][52]=='1';
	I54->Checked=mask[6][53]=='1';
	I55->Checked=mask[6][54]=='1';
	I56->Checked=mask[6][55]=='1';
	I02->Checked=mask[6][1]=='1';
	I12->Checked=mask[6][11]=='1';
	S01->Checked=mask[4][ 0]=='1';
	S24->Checked=mask[4][23]=='1';
	S25->Checked=mask[4][24]=='1';
	S26->Checked=mask[4][25]=='1';
	
	UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TCodeOptDialog::BtnOkClick(TObject *Sender)
{
	char mask[7][MAXCODE+1]={""};
	
	for (int i=0;i<7;i++) for (int j=0;j<MAXCODE;j++) mask[i][j]='0';
	if (G01->Checked) mask[0][ 0]='1';
	if (G02->Checked) mask[0][ 1]='1';
	if (G03->Checked) mask[0][ 2]='1';
	if (G04->Checked) mask[0][ 3]='1';
	if (G05->Checked) mask[0][ 4]='1';
	if (G06->Checked) mask[0][ 5]='1';
	if (G07->Checked) mask[0][ 6]='1';
	if (G08->Checked) mask[0][ 7]='1';
	if (G12->Checked) mask[0][11]='1';
	if (G14->Checked) mask[0][13]='1';
	if (G15->Checked) mask[0][14]='1';
	if (G16->Checked) mask[0][15]='1';
	if (G17->Checked) mask[0][16]='1';
	if (G18->Checked) mask[0][17]='1';
	if (G19->Checked) mask[0][18]='1';
	if (G20->Checked) mask[0][19]='1';
	if (G21->Checked) mask[0][20]='1';
	if (G22->Checked) mask[0][21]='1';
	if (G23->Checked) mask[0][22]='1';
	if (G24->Checked) mask[0][23]='1';
	if (G25->Checked) mask[0][24]='1';
	if (G26->Checked) mask[0][25]='1';
	if (R01->Checked) mask[1][ 0]='1';
	if (R02->Checked) mask[1][ 1]='1';
	if (R14->Checked) mask[1][13]='1';
	if (R19->Checked) mask[1][18]='1';
	if (R44->Checked) mask[1][43]='1';
	if (R45->Checked) mask[1][44]='1';
	if (R46->Checked) mask[1][45]='1';
	if (R66->Checked) mask[1][65]='1'; //
	if (R67->Checked) mask[1][66]='1'; //
	if (R68->Checked) mask[1][67]='1'; //
	if (R30->Checked) mask[1][29]='1'; //
	if (R31->Checked) mask[1][30]='1'; //
	if (R33->Checked) mask[1][32]='1'; //
	if (E01->Checked) mask[2][ 0]='1';
	if (E10->Checked) mask[2][ 9]='1';
	if (E11->Checked) mask[2][10]='1';
	if (E12->Checked) mask[2][11]='1';
	if (E13->Checked) mask[2][12]='1';
	if (E24->Checked) mask[2][23]='1';
	if (E25->Checked) mask[2][24]='1';
	if (E26->Checked) mask[2][25]='1';
	if (E27->Checked) mask[2][26]='1';
	if (E28->Checked) mask[2][27]='1';
	if (E29->Checked) mask[2][28]='1';
	if (E30->Checked) mask[2][29]='1';
	if (E31->Checked) mask[2][30]='1';
	if (E32->Checked) mask[2][31]='1';
	if (E33->Checked) mask[2][32]='1';
	if (E34->Checked) mask[2][33]='1';
	if (E37->Checked) mask[2][36]='1';
	if (E38->Checked) mask[2][37]='1';
	if (E39->Checked) mask[2][38]='1';
	if (J01->Checked) mask[3][ 0]='1';
	if (J07->Checked) mask[3][ 6]='1';
	if (J08->Checked) mask[3][ 7]='1';
	if (J09->Checked) mask[3][ 8]='1';
	if (J11->Checked) mask[3][10]='1';
	if (J12->Checked) mask[3][11]='1';
	if (J13->Checked) mask[3][12]='1';
	if (J16->Checked) mask[3][15]='1';
	if (J17->Checked) mask[3][16]='1';
	if (J18->Checked) mask[3][17]='1';
	if (J24->Checked) mask[3][23]='1';
	if (J25->Checked) mask[3][24]='1';
	if (J26->Checked) mask[3][25]='1';
	if (J57->Checked) mask[3][56]='1'; //
	if (J58->Checked) mask[3][57]='1'; //
	if (J59->Checked) mask[3][58]='1'; //
	if (J60->Checked) mask[3][59]='1'; //
	if (J34->Checked) mask[3][33]='1'; //
	if (J35->Checked) mask[3][34]='1';
	if (J36->Checked) mask[3][35]='1';
	if (J33->Checked) mask[3][32]='1';
	if (C40->Checked) mask[5][39]='1'; //
	if (C41->Checked) mask[5][40]='1'; //
	if (C18->Checked) mask[5][17]='1'; //
	if (C27->Checked) mask[5][26]='1';
	if (C28->Checked) mask[5][27]='1';
	if (C29->Checked) mask[5][28]='1';
	if (C42->Checked) mask[5][41]='1';
	if (C43->Checked) mask[5][42]='1';
	if (C33->Checked) mask[5][32]='1';
	if (C56->Checked) mask[5][55]='1'; //
	if (C02->Checked) mask[5][ 1]='1'; //
	if (C12->Checked) mask[5][11]='1'; //
	if (C07->Checked) mask[5][ 6]='1'; //
	if (C08->Checked) mask[5][ 7]='1'; //
	if (C13->Checked) mask[5][12]='1'; //
	if (C57->Checked) mask[5][56]='1'; //
	if (C58->Checked) mask[5][57]='1'; //
	if (C26->Checked) mask[5][25]='1'; //
	if (C61->Checked) mask[5][60]='1'; //
	if (C62->Checked) mask[5][61]='1'; //
	if (C63->Checked) mask[5][62]='1'; //
	if (C64->Checked) mask[5][63]='1'; //
	if (C65->Checked) mask[5][64]='1'; //
	if (C39->Checked) mask[5][38]='1'; //
	if (C69->Checked) mask[5][68]='1'; //
	if (C70->Checked) mask[5][69]='1'; //
	if (C34->Checked) mask[5][33]='1'; //
	if (I49->Checked) mask[6][48]='1';
	if (I50->Checked) mask[6][49]='1';
	if (I51->Checked) mask[6][50]='1';
	if (I26->Checked) mask[6][25]='1';
	if (I52->Checked) mask[6][51]='1';
	if (I53->Checked) mask[6][52]='1';
	if (I54->Checked) mask[6][53]='1';
	if (I55->Checked) mask[6][54]='1';
	if (I56->Checked) mask[6][55]='1';
	if (I02->Checked) mask[6][ 1]='1';
	if (I12->Checked) mask[6][11]='1';
	if (S01->Checked) mask[4][ 0]='1';
	if (S24->Checked) mask[4][23]='1';
	if (S25->Checked) mask[4][24]='1';
	if (S26->Checked) mask[4][25]='1';
	for (int i=0;i<7;i++) ConvOptDialog->CodeMask[i]=mask[i];
}
//---------------------------------------------------------------------------
void __fastcall TCodeOptDialog::BtnSetAllClick(TObject *Sender)
{
	int set=BtnSetAll->Caption=="Set All";
	
	G01->Checked=set;
	G02->Checked=set;
	G03->Checked=set;
	G04->Checked=set;
	G05->Checked=set;
	G06->Checked=set;
	G07->Checked=set;
	G08->Checked=set;
	G12->Checked=set;
	G14->Checked=set;
	G15->Checked=set;
	G16->Checked=set;
	G17->Checked=set;
	G18->Checked=set;
	G19->Checked=set;
	G20->Checked=set;
	G21->Checked=set;
	G22->Checked=set;
	G23->Checked=set;
	G24->Checked=set;
	G25->Checked=set;
	G26->Checked=set;
	R01->Checked=set;
	R02->Checked=set;
	R14->Checked=set;
	R19->Checked=set;
	R44->Checked=set;
	R45->Checked=set;
	R46->Checked=set;
	R66->Checked=set; //
	R67->Checked=set; //
	R68->Checked=set; //
	R30->Checked=set; //
	R31->Checked=set; //
	R33->Checked=set; //
	E01->Checked=set;
	E10->Checked=set;
	E11->Checked=set;
	E12->Checked=set;
	E13->Checked=set;
	E24->Checked=set;
	E25->Checked=set;
	E26->Checked=set;
	E27->Checked=set;
	E28->Checked=set;
	E29->Checked=set;
	E30->Checked=set;
	E31->Checked=set;
	E32->Checked=set;
	E33->Checked=set;
	E34->Checked=set;
	E37->Checked=set;
	E38->Checked=set;
	E39->Checked=set;
	J01->Checked=set;
	J07->Checked=set;
	J08->Checked=set;
	J09->Checked=set;
	J11->Checked=set;
	J12->Checked=set;
	J13->Checked=set;
	J16->Checked=set;
	J17->Checked=set;
	J18->Checked=set;
	J24->Checked=set;
	J25->Checked=set;
	J26->Checked=set;
	J57->Checked=set; //
	J58->Checked=set; //
	J59->Checked=set; //
	J60->Checked=set; //
	J34->Checked=set; //
	J35->Checked=set;
	J36->Checked=set;
	J33->Checked=set;
	C40->Checked=set; //
	C41->Checked=set; //
	C18->Checked=set; //
	C27->Checked=set;
	C28->Checked=set;
	C29->Checked=set;
	C42->Checked=set;
	C43->Checked=set;
	C33->Checked=set;
	C56->Checked=set; //
	C02->Checked=set; //
	C12->Checked=set; //
	C07->Checked=set; //
	C08->Checked=set; //
	C13->Checked=set; //
	C57->Checked=set; //
	C58->Checked=set; //
	C26->Checked=set; //
	C61->Checked=set; //
	C62->Checked=set; //
	C63->Checked=set; //
	C64->Checked=set; //
	C65->Checked=set; //
	C39->Checked=set; //
	C69->Checked=set; //
	C70->Checked=set; //
	C34->Checked=set; //
	I49->Checked=set;
	I50->Checked=set;
	I51->Checked=set;
	I26->Checked=set;
	I52->Checked=set;
	I53->Checked=set;
	I54->Checked=set;
	I55->Checked=set;
	I56->Checked=set;
	I02->Checked=set;
	I12->Checked=set;
	S01->Checked=set;
	S24->Checked=set;
	S25->Checked=set;
	S26->Checked=set;
	BtnSetAll->Caption=BtnSetAll->Caption=="Set All"?"Unset All":"Set All";
}
//---------------------------------------------------------------------------
static int testsyscode(int sys, const char *code, int nsys, int freqtype)
{
  if ((nsys & sys) == 0) return 0;
  int idx = code2idx(sys, obs2code(code));
  return idx >= 0 && (freqtype & (1 << idx)) != 0;
}
void __fastcall TCodeOptDialog::UpdateEnable(void)
{
    G01->Enabled = testsyscode(SYS_GPS, "1C", NavSys, FreqType);
    G02->Enabled = testsyscode(SYS_GPS, "1P", NavSys, FreqType);
    G03->Enabled = testsyscode(SYS_GPS, "1W", NavSys, FreqType);
    G04->Enabled = testsyscode(SYS_GPS, "1Y", NavSys, FreqType);
    G05->Enabled = testsyscode(SYS_GPS, "1M", NavSys, FreqType);
    G06->Enabled = testsyscode(SYS_GPS, "1N", NavSys, FreqType);
    G07->Enabled = testsyscode(SYS_GPS, "1S", NavSys, FreqType);
    G08->Enabled = testsyscode(SYS_GPS, "1L", NavSys, FreqType);
    G12->Enabled = testsyscode(SYS_GPS, "1X", NavSys, FreqType);
    G14->Enabled = testsyscode(SYS_GPS, "2C", NavSys, FreqType);
    G15->Enabled = testsyscode(SYS_GPS, "2D", NavSys, FreqType);
    G16->Enabled = testsyscode(SYS_GPS, "2S", NavSys, FreqType);
    G17->Enabled = testsyscode(SYS_GPS, "2L", NavSys, FreqType);
    G18->Enabled = testsyscode(SYS_GPS, "2X", NavSys, FreqType);
    G19->Enabled = testsyscode(SYS_GPS, "2P", NavSys, FreqType);
    G20->Enabled = testsyscode(SYS_GPS, "2W", NavSys, FreqType);
    G21->Enabled = testsyscode(SYS_GPS, "2Y", NavSys, FreqType);
    G22->Enabled = testsyscode(SYS_GPS, "2M", NavSys, FreqType);
    G23->Enabled = testsyscode(SYS_GPS, "2N", NavSys, FreqType);
    G24->Enabled = testsyscode(SYS_GPS, "5I", NavSys, FreqType);
    G25->Enabled = testsyscode(SYS_GPS, "5Q", NavSys, FreqType);
    G26->Enabled = testsyscode(SYS_GPS, "5X", NavSys, FreqType);

    R01->Enabled = testsyscode(SYS_GLO, "1C", NavSys, FreqType);
    R02->Enabled = testsyscode(SYS_GLO, "1P", NavSys, FreqType);
    R14->Enabled = testsyscode(SYS_GLO, "2C", NavSys, FreqType);
    R19->Enabled = testsyscode(SYS_GLO, "2P", NavSys, FreqType);
    R30->Enabled = testsyscode(SYS_GLO, "6A", NavSys, FreqType);
    R31->Enabled = testsyscode(SYS_GLO, "6B", NavSys, FreqType);
    R33->Enabled = testsyscode(SYS_GLO, "6X", NavSys, FreqType);
    R44->Enabled = testsyscode(SYS_GLO, "3I", NavSys, FreqType);
    R45->Enabled = testsyscode(SYS_GLO, "3Q", NavSys, FreqType);
    R46->Enabled = testsyscode(SYS_GLO, "3X", NavSys, FreqType);
    R66->Enabled = testsyscode(SYS_GLO, "4A", NavSys, FreqType);
    R67->Enabled = testsyscode(SYS_GLO, "4B", NavSys, FreqType);
    R68->Enabled = testsyscode(SYS_GLO, "4X", NavSys, FreqType);

    E01->Enabled = testsyscode(SYS_GAL, "1C", NavSys, FreqType);
    E10->Enabled = testsyscode(SYS_GAL, "1A", NavSys, FreqType);
    E11->Enabled = testsyscode(SYS_GAL, "1B", NavSys, FreqType);
    E12->Enabled = testsyscode(SYS_GAL, "1X", NavSys, FreqType);
    E13->Enabled = testsyscode(SYS_GAL, "1Z", NavSys, FreqType);
    E24->Enabled = testsyscode(SYS_GAL, "5I", NavSys, FreqType);
    E25->Enabled = testsyscode(SYS_GAL, "5Q", NavSys, FreqType);
    E26->Enabled = testsyscode(SYS_GAL, "5X", NavSys, FreqType);
    E27->Enabled = testsyscode(SYS_GAL, "7I", NavSys, FreqType);
    E28->Enabled = testsyscode(SYS_GAL, "7Q", NavSys, FreqType);
    E29->Enabled = testsyscode(SYS_GAL, "7X", NavSys, FreqType);
    E30->Enabled = testsyscode(SYS_GAL, "6A", NavSys, FreqType);
    E31->Enabled = testsyscode(SYS_GAL, "6B", NavSys, FreqType);
    E32->Enabled = testsyscode(SYS_GAL, "6C", NavSys, FreqType);
    E33->Enabled = testsyscode(SYS_GAL, "6X", NavSys, FreqType);
    E34->Enabled = testsyscode(SYS_GAL, "6Z", NavSys, FreqType);
    E37->Enabled = testsyscode(SYS_GAL, "8I", NavSys, FreqType);
    E38->Enabled = testsyscode(SYS_GAL, "8Q", NavSys, FreqType);
    E39->Enabled = testsyscode(SYS_GAL, "8X", NavSys, FreqType);

    J01->Enabled = testsyscode(SYS_QZS, "1C", NavSys, FreqType);
    J07->Enabled = testsyscode(SYS_QZS, "1S", NavSys, FreqType);
    J08->Enabled = testsyscode(SYS_QZS, "1L", NavSys, FreqType);
    J09->Enabled = testsyscode(SYS_QZS, "1E", NavSys, FreqType);
    J11->Enabled = testsyscode(SYS_QZS, "1B", NavSys, FreqType);
    J12->Enabled = testsyscode(SYS_QZS, "1X", NavSys, FreqType);
    J13->Enabled = testsyscode(SYS_QZS, "1Z", NavSys, FreqType);
    J16->Enabled = testsyscode(SYS_QZS, "2S", NavSys, FreqType);
    J17->Enabled = testsyscode(SYS_QZS, "2L", NavSys, FreqType);
    J18->Enabled = testsyscode(SYS_QZS, "2X", NavSys, FreqType);
    J24->Enabled = testsyscode(SYS_QZS, "5I", NavSys, FreqType);
    J25->Enabled = testsyscode(SYS_QZS, "5Q", NavSys, FreqType);
    J26->Enabled = testsyscode(SYS_QZS, "5X", NavSys, FreqType);
    J33->Enabled = testsyscode(SYS_QZS, "6X", NavSys, FreqType);
    J34->Enabled = testsyscode(SYS_QZS, "6Z", NavSys, FreqType);
    J35->Enabled = testsyscode(SYS_QZS, "6S", NavSys, FreqType);
    J36->Enabled = testsyscode(SYS_QZS, "6L", NavSys, FreqType);
    J57->Enabled = testsyscode(SYS_QZS, "5D", NavSys, FreqType);
    J58->Enabled = testsyscode(SYS_QZS, "5P", NavSys, FreqType);
    J59->Enabled = testsyscode(SYS_QZS, "5Z", NavSys, FreqType);
    J60->Enabled = testsyscode(SYS_QZS, "6E", NavSys, FreqType);

    S01->Enabled = testsyscode(SYS_SBS, "1C", NavSys, FreqType);
    S24->Enabled = testsyscode(SYS_SBS, "5I", NavSys, FreqType);
    S25->Enabled = testsyscode(SYS_SBS, "5Q", NavSys, FreqType);
    S26->Enabled = testsyscode(SYS_SBS, "5X", NavSys, FreqType);

    C02->Enabled = testsyscode(SYS_BDS3, "1P", NavSys, FreqType);
    C07->Enabled = testsyscode(SYS_BDS3, "1S", NavSys, FreqType);
    C08->Enabled = testsyscode(SYS_BDS3, "1L", NavSys, FreqType);
    C12->Enabled = testsyscode(SYS_BDS3, "1X", NavSys, FreqType);
    C13->Enabled = testsyscode(SYS_BDS3, "1Z", NavSys, FreqType);
    C18->Enabled = testsyscode(SYS_BDS2, "2X", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "2X", NavSys, FreqType);
    C26->Enabled = testsyscode(SYS_BDS3, "5X", NavSys, FreqType);
    C27->Enabled = testsyscode(SYS_BDS2, "7I", NavSys, FreqType);
    C28->Enabled = testsyscode(SYS_BDS2, "7Q", NavSys, FreqType);
    C29->Enabled = testsyscode(SYS_BDS2, "7X", NavSys, FreqType);
    C33->Enabled = testsyscode(SYS_BDS2, "6X", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "6X", NavSys, FreqType);
    C39->Enabled = testsyscode(SYS_BDS3, "8X", NavSys, FreqType);
    C40->Enabled = testsyscode(SYS_BDS2, "2I", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "2I", NavSys, FreqType);
    C41->Enabled = testsyscode(SYS_BDS2, "2Q", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "2Q", NavSys, FreqType);
    C42->Enabled = testsyscode(SYS_BDS2, "6I", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "6I", NavSys, FreqType);
    C43->Enabled = testsyscode(SYS_BDS2, "6Q", NavSys, FreqType) ||
        testsyscode(SYS_BDS3, "6Q", NavSys, FreqType);
    C56->Enabled = testsyscode(SYS_BDS3, "1D", NavSys, FreqType);
    C57->Enabled = testsyscode(SYS_BDS3, "5D", NavSys, FreqType);
    C58->Enabled = testsyscode(SYS_BDS3, "5P", NavSys, FreqType);
    C61->Enabled = testsyscode(SYS_BDS3, "7D", NavSys, FreqType);
    C62->Enabled = testsyscode(SYS_BDS3, "7P", NavSys, FreqType);
    C63->Enabled = testsyscode(SYS_BDS3, "7Z", NavSys, FreqType);
    C64->Enabled = testsyscode(SYS_BDS3, "8D", NavSys, FreqType);
    C65->Enabled = testsyscode(SYS_BDS3, "8P", NavSys, FreqType);
    C69->Enabled = testsyscode(SYS_BDS3, "6D", NavSys, FreqType);
    C70->Enabled = testsyscode(SYS_BDS3, "6P", NavSys, FreqType);
    C34->Enabled = testsyscode(SYS_BDS3, "6Z", NavSys, FreqType);

    I26->Enabled = testsyscode(SYS_IRN, "5X", NavSys, FreqType);
    I49->Enabled = testsyscode(SYS_IRN, "5A", NavSys, FreqType);
    I50->Enabled = testsyscode(SYS_IRN, "5B", NavSys, FreqType);
    I51->Enabled = testsyscode(SYS_IRN, "5C", NavSys, FreqType);
    I52->Enabled = testsyscode(SYS_IRN, "9A", NavSys, FreqType);
    I53->Enabled = testsyscode(SYS_IRN, "9B", NavSys, FreqType);
    I54->Enabled = testsyscode(SYS_IRN, "9C", NavSys, FreqType);
    I55->Enabled = testsyscode(SYS_IRN, "9X", NavSys, FreqType);
    I56->Enabled = testsyscode(SYS_IRN, "1D", NavSys, FreqType);
    I02->Enabled = testsyscode(SYS_IRN, "1P", NavSys, FreqType);
    I12->Enabled = testsyscode(SYS_IRN, "1X", NavSys, FreqType);
}
//---------------------------------------------------------------------------

