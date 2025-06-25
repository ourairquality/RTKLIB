//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "freqdlg.h"
#include "rtklib.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"

TFreqDialog *FreqDialog;
//---------------------------------------------------------------------------
__fastcall TFreqDialog::TFreqDialog(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TFreqDialog::FormShow(TObject *Sender)
{
  const int sys[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_BDS2, SYS_BDS3, SYS_IRN, SYS_SBS};
  TPanel *panels[][MAXFREQ] = {{PanelG1, PanelG2, PanelG3, PanelG4, PanelG5, PanelG6},
                               {PanelR1, PanelR2, PanelR3, PanelR4, PanelR5, PanelR6},
                               {PanelE1, PanelE2, PanelE3, PanelE4, PanelE5, PanelE6},
                               {PanelQ1, PanelQ2, PanelQ3, PanelQ4, PanelQ5, PanelQ6},
                               {PanelB21, PanelB22, PanelB23, PanelB24, PanelB25, PanelB26},
                               {PanelB31, PanelB32, PanelB33, PanelB34, PanelB35, PanelB36},
                               {PanelI1, PanelI2, PanelI3, PanelI4, PanelI5, PanelI6},
                               {PanelS1, PanelS2, PanelS3, PanelS4, PanelS5, PanelS6}};
  TPanel *panelsMHz[][MAXFREQ] = {{PanelG1MHz, PanelG2MHz, PanelG3MHz, PanelG4MHz, PanelG5MHz, PanelG6MHz},
                                  {PanelR1MHz, PanelR2MHz, PanelR3MHz, PanelR4MHz, PanelR5MHz, PanelR6MHz},
                                  {PanelE1MHz, PanelE2MHz, PanelE3MHz, PanelE4MHz, PanelE5MHz, PanelE6MHz},
                                  {PanelQ1MHz, PanelQ2MHz, PanelQ3MHz, PanelQ4MHz, PanelQ5MHz, PanelQ6MHz},
                                  {PanelB21MHz, PanelB22MHz, PanelB23MHz, PanelB24MHz, PanelB25MHz, PanelB26MHz},
                                  {PanelB31MHz, PanelB32MHz, PanelB33MHz, PanelB34MHz, PanelB35MHz, PanelB36MHz},
                                  {PanelI1MHz, PanelI2MHz, PanelI3MHz, PanelI4MHz, PanelI5MHz, PanelI6MHz},
                                  {PanelS1MHz, PanelS2MHz, PanelS3MHz, PanelS4MHz, PanelS5MHz, PanelS6MHz}};

  for (int idx = 0; idx < MAXFREQ; idx++) {
    for (int i = 0; i < 8; i++) {
      int band = idx2band(sys[i], idx);
      if (band) {
        char *codepri = getcodepriorities(sys[i], band);
        const char *name = getcodebandname(sys[i], band);
        double freq = band2freq(sys[i], band, 0);
        AnsiString s;
        panels[i][idx]->Caption = s.sprintf("%4s %d%s", name, band, codepri);
        panelsMHz[i][idx]->Caption = s.sprintf("%8.3f", freq * 1e-6);
      } else {
        panels[i][idx]->Caption = "-";
        panelsMHz[i][idx]->Caption = "";
      }
    }
  }
}
//---------------------------------------------------------------------------
