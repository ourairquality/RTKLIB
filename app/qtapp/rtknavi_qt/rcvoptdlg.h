//---------------------------------------------------------------------------
#ifndef rcvoptdlgH
#define rcvoptdlgH
//---------------------------------------------------------------------------

#include <ui_rcvoptdlg.h>

#include <QDialog>
//---------------------------------------------------------------------------
class RcvOptDialog : public QDialog, private Ui::RcvOptDialog {
  Q_OBJECT

 protected:
  void showEvent(QShowEvent*);

 public slots:
  void BtnOkClick();

 public:
  QString Option;

  explicit RcvOptDialog(QWidget* parent);
};
//---------------------------------------------------------------------------
#endif
