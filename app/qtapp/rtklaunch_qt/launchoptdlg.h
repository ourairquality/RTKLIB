//---------------------------------------------------------------------------

#ifndef launchoptdlgH
#define launchoptdlgH
//---------------------------------------------------------------------------
#include <QDialog>

#include "ui_launchoptdlg.h"

class QShowEvent;
//---------------------------------------------------------------------------
class LaunchOptDialog : public QDialog, private Ui::LaunchOptDialog {
  Q_OBJECT
 public slots:
  void BtnOkClick();

 protected:
  void showEvent(QShowEvent*);

 public:
  LaunchOptDialog(QWidget* parent);
};
//---------------------------------------------------------------------------
#endif
