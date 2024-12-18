//---------------------------------------------------------------------------
#ifndef infiledlgH
#define infiledlgH
//---------------------------------------------------------------------------

#include <QDialog>

namespace Ui {
class InputFileDialog;
}

//---------------------------------------------------------------------------
class InputFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InputFileDialog(QWidget* parent);

    void setHistory(int i, const QString &history);
    const QString &getHistory(int i);

    void setPath(int n, const QString &);
    QString getPath(int n);

public slots:
    void  selectFile1();
    void  selectFile2();
    void  selectFile3();
    void  selectFile4();
    void  selectFile5();
    void  selectFile6();

private:
    QString paths[6];

    Ui::InputFileDialog *ui;
};
#endif
