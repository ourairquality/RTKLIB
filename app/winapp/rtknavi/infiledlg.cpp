//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop

#include "rtklib.h"
#include "navimain.h"
#include "infiledlg.h"
#include "keydlg.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TInputFileDialog *InputFileDialog;
//---------------------------------------------------------------------------
__fastcall TInputFileDialog::TInputFileDialog(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::FormShow(TObject *Sender)
{
	AnsiString s;
	FilePath1 ->Text     =Paths[0];
	FilePath2 ->Text     =Paths[1];
	FilePath3 ->Text     =Paths[2];
	FilePath4 ->Text     =Paths[3];
	FilePath5 ->Text     =Paths[4];
	FilePath6 ->Text     =Paths[5];
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnOkClick(TObject *Sender)
{
	Paths[0]=FilePath1->Text;
	Paths[1]=FilePath2->Text;
	Paths[2]=FilePath3->Text;
	Paths[3]=FilePath4->Text;
	Paths[4]=FilePath5->Text;
	Paths[5]=FilePath6->Text;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile1Click(TObject *Sender)
{
        OpenDialog->FileName=FilePath1->Text;
	if (!OpenDialog->Execute()) return;
	FilePath1->Text=OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile2Click(TObject *Sender)
{
	OpenDialog->FileName=FilePath2->Text;
	if (!OpenDialog->Execute()) return;
	FilePath2->Text=OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile3Click(TObject *Sender)
{
	OpenDialog->FileName=FilePath3->Text;
	if (!OpenDialog->Execute()) return;
	FilePath3->Text=OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile4Click(TObject *Sender)
{
	OpenDialog->FileName=FilePath4->Text;
	if (!OpenDialog->Execute()) return;
	FilePath4->Text=OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile5Click(TObject *Sender)
{
	OpenDialog->FileName=FilePath5->Text;
	if (!OpenDialog->Execute()) return;
	FilePath5->Text=OpenDialog->FileName;
}
//---------------------------------------------------------------------------
void __fastcall TInputFileDialog::BtnFile6Click(TObject *Sender)
{
	OpenDialog->FileName=FilePath6->Text;
	if (!OpenDialog->Execute()) return;
	FilePath6->Text=OpenDialog->FileName;
}
// callback on button keyword -----------------------------------------------
void __fastcall TInputFileDialog::BtnKeywordClick(TObject *Sender)
{
	KeyDialog->Flag = 0;
	KeyDialog->Show();
}
//---------------------------------------------------------------------------

