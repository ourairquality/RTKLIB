//---------------------------------------------------------------------------
#ifndef startdlgH
#define startdlgH
//---------------------------------------------------------------------------
#include <QDialog>

#include "rtklib.h"
#include "ui_startdlg.h"
class QShowEvent;

//---------------------------------------------------------------------------
class StartDialog : public QDialog, private Ui::StartDialog {
  Q_OBJECT

 protected:
  void showEvent(QShowEvent *);

 public slots:
  void BtnOkClick();
  void BtnFileTimeClick();

 public:
  gtime_t Time;
  QString FileName;

  explicit StartDialog(QWidget *parent = NULL);
};
//---------------------------------------------------------------------------
#endif
