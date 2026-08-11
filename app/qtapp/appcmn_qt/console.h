//---------------------------------------------------------------------------
#ifndef consoleH
#define consoleH
//---------------------------------------------------------------------------
#include <QDialog>
//---------------------------------------------------------------------------

namespace Ui {
    class Console;
}

class Console : public QDialog
{
    Q_OBJECT

public:
    explicit Console(QWidget* parent);
    void  addMessage(uint8_t *msg, int n);
    void  setFont(QFont font);

protected slots:
    void  btnClearClicked();
    void  btnAsciiClicked();
    void  btnHexClicked();
    void  btnDownClicked();

private:
    QStringList consoleBuffer;
    Ui::Console *ui;
};
//---------------------------------------------------------------------------
#endif
