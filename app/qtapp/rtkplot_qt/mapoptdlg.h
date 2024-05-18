//---------------------------------------------------------------------------
#ifndef mapoptdlgH
#define mapoptdlgH
//---------------------------------------------------------------------------
#include <QDialog>

#include "ui_mapoptdlg.h"

class QKeyEvent;
class QShowEvent;

//---------------------------------------------------------------------------
class MapOptDialog : public QDialog, private Ui::MapAreaDialog {
  Q_OBJECT

 protected:
  void showEvent(QShowEvent*);

 public slots:
  void BtnCloseClick();
  void BtnUpdateClick();
  void BtnSaveClick();

  void BtnCenterClick();
  void ScaleEqClick();

 private:
  void UpdateMap(void);
  void UpdatePlot(void);
  void UpdateEnable(void);

 public:
  MapOptDialog(QWidget* parent);
  void UpdateField(void);
};

//---------------------------------------------------------------------------
#endif
