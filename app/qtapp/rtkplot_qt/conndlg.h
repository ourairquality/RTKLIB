//---------------------------------------------------------------------------
#ifndef conndlgH
#define conndlgH
//---------------------------------------------------------------------------

#define MAXHIST		10

#include <QDialog>

namespace Ui {
class ConnectDialog;
}

class QShowEvent;

//---------------------------------------------------------------------------
class ConnectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConnectDialog(QWidget *parent=NULL);

    void setStreamType(int stream, int type);
    int getStreamType(int stream);

    void setStreamFormat(int stream, int format);
    int getStreamFormat(int stream);

    void setCommands(int stream, int type, const QString &);
    QString getCommands(int stream, int type);

    void setCommandsEnabled(int stream, int type, bool ena);
    bool getCommandsEnabled(int stream, int type);

    void setTimeFormat(int);
    int getTimeFormat();

    void setDegFormat(int);
    int getDegFormat();

    void setTLSSvrCertFile(QString file);
    QString getTLSSvrCertFile();
    void setTLSSvrKeyFile(QString file);
    QString getTLSSvrKeyFile();
    void setTLSSvrCAFile(QString file);
    QString getTLSSvrCAFile();
    void setTLSSvrCADir(QString file);
    QString getTLSSvrCADir();
    void setTLSCliCertFile(QString file);
    QString getTLSCliCertFile();
    void setTLSCliKeyFile(QString file);
    QString getTLSCliKeyFile();
    void setTLSCliCAFile(QString file);
    QString getTLSCliCAFile();
    void setTLSCliCADir(QString file);
    QString getTLSCliCADir();

    void setTimeoutTime(int);
    int getTimeoutTime();

    void setReconnectTime(int);
    int getReconnectTime();

    void setFieldSeparator(const QString &);
    QString getFieldSeparator();

    void setPath(int stream, int type, const QString &);
    QString getPath(int stream, int type);

    void setHistory(int i, const QString &history);
    const QString &getHistory(int i);

protected slots:
    void selectOptionsStream1();
    void selectOptionsStream2();
    void selectCommandsStream1();
    void selectCommandsStream2();
    void updateEnable();
    void tlsSvrCertFileSelect();
    void tlsSvrKeyFileSelect();
    void tlsSvrCAFileSelect();
    void tlsSvrCADirSelect();
    void tlsCliCertFileSelect();
    void tlsCliKeyFileSelect();
    void tlsCliCAFileSelect();
    void tlsCliCADirSelect();

protected:
    int commandEnable[2][2];
    QString commands[2][2];
    QString paths[2][7];
    QString history[MAXHIST];

private:
    void serialOptionsStream(int stream, int opt, unsigned i);
    void tcpOption(int stream, int opt, unsigned i);
    void fileOption(int stream, int opt, unsigned i);

    Ui::ConnectDialog *ui;

};
//---------------------------------------------------------------------------
#endif
