//---------------------------------------------------------------------------
#ifndef svroptdlgH
#define svroptdlgH
//---------------------------------------------------------------------------
#include <QDialog>

namespace Ui {
class SvrOptDialog;
}

class QShowEvent;
class RefDialog;
//---------------------------------------------------------------------------
class SvrOptDialog : public QDialog
{
    Q_OBJECT

public slots:
    void accept();
    void positionSelect();
    void localDirectorySelect();
    void logFileSelect();
    void tlsSvrCertFileSelect();
    void tlsSvrKeyFileSelect();
    void tlsSvrCAFileSelect();
    void tlsSvrCADirSelect();
    void tlsCliCertFileSelect();
    void tlsCliKeyFileSelect();
    void tlsCliCAFileSelect();
    void tlsCliCADirSelect();

protected:
    void showEvent(QShowEvent*);
    RefDialog *refDialog;

private:
    void updateEnable();
    Ui::SvrOptDialog *ui;

public:
    QString stationPositionFile, exeDirectory, localDirectory, proxyAddress;
    QString antennaType, receiverType, logFile;
    QString tlsSvrCertFile, tlsSvrKeyFile, tlsSvrCAFile, tlsSvrCADir;
    QString tlsCliCertFile, tlsCliKeyFile, tlsCliCAFile, tlsCliCADir;
    int serverOptions[6], traceLevel, nmeaRequest, fileSwapMargin, stationId, stationSelect, relayBack;
    int progressBarRange;
    double antennaPosition[3], antennaOffsets[3];

    explicit SvrOptDialog(QWidget *parent);
};
//---------------------------------------------------------------------------
#endif
