//---------------------------------------------------------------------------

#ifndef mapviewoptH
#define mapviewoptH
//---------------------------------------------------------------------------
#include <QDialog>

#include "ui_mapviewopt.h"

//---------------------------------------------------------------------------
class MapViewOptDialog : public QDialog, private Ui::MapViewOpt {
  Q_OBJECT

 protected:
  void showEvent(QShowEvent*);

 public slots:
  void BtnOkClick();
  void BtnNotesClick();

 public:
  QString MapStrs[6][3];
  QString ApiKey;
  MapViewOptDialog(QWidget* parent);
};

//---------------------------------------------------------------------------
#endif
