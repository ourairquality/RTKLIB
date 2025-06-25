//---------------------------------------------------------------------------

#include "freqdlg.h"

#include "ui_freqdlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
FreqDialog::FreqDialog(QWidget* parent)
    : QDialog(parent), ui(new Ui::FreqDialog)
{
    ui->setupUi(this);

    for (int idx = 0; idx < MAXFREQ; idx++) {
      const int sys[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_BDS2, SYS_BDS3, SYS_IRN, SYS_SBS};
      for (int i = 0; i < 8; i++) {
        int band = idx2band(sys[i], idx);
        if (band) {
          char *codepri = getcodepriorities(sys[i], band);
          const char *name = getcodebandname(sys[i], band);
          double freq = band2freq(sys[i], band, 0);
          ui->tableWidget->item(i, idx * 3)->setText(QString("%1").arg(name));
          ui->tableWidget->item(i, idx * 3 + 1)->setText(QString("%1%2").arg(band).arg(codepri));
          ui->tableWidget->item(i, idx * 3 + 2)->setText(QString("%1").arg(freq * 1e-6, 8, 'f', 3));
        } else {
          ui->tableWidget->item(i, idx * 3)->setText("-");
          ui->tableWidget->item(i, idx * 3 + 1)->setText("");
          ui->tableWidget->item(i, idx * 3 + 2)->setText("");
        }
      }
    }

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &FreqDialog::accept);
}
//---------------------------------------------------------------------------
