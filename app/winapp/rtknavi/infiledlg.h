//---------------------------------------------------------------------------
#ifndef infiledlgH
#define infiledlgH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include <Dialogs.hpp>
#include <Buttons.hpp>

#include "rtklib.h"

//---------------------------------------------------------------------------
class TInputFileDialog : public TForm
{
__published:
	TLabel *LabelF1;
	TEdit *FilePath1;
	TButton *BtnFile1;
	TEdit *FilePath2;
	TButton *BtnFile2;
	TEdit *FilePath3;
	TButton *BtnFile3;
	TEdit *FilePath4;
	TButton *BtnFile4;
	TEdit *FilePath5;
	TButton *BtnFile5;
	TEdit *FilePath6;
	TButton *BtnFile6;
	TButton *BtnOk;
	TButton *BtnCancel;
	TOpenDialog *OpenDialog;
	TSpeedButton *BtnKeyword;

	void __fastcall BtnOkClick(TObject *Sender);
	void __fastcall FormShow(TObject *Sender);
	void __fastcall BtnFile1Click(TObject *Sender);
	void __fastcall BtnFile2Click(TObject *Sender);
	void __fastcall BtnFile3Click(TObject *Sender);
	void __fastcall BtnFile4Click(TObject *Sender);
	void __fastcall BtnFile5Click(TObject *Sender);
	void __fastcall BtnFile6Click(TObject *Sender);
	void __fastcall BtnKeywordClick(TObject *Sender);
private:
	AnsiString __fastcall GetFilePath(AnsiString path);
	AnsiString __fastcall SetFilePath(AnsiString path);
public:
	AnsiString Paths[MAXINFILES];
	__fastcall TInputFileDialog(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TInputFileDialog *InputFileDialog;
//---------------------------------------------------------------------------
#endif
