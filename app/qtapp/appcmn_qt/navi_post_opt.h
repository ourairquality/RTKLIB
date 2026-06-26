//---------------------------------------------------------------------------
#ifndef navioptH
#define navioptH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QRegularExpression>

#include "rtklib.h"

namespace Ui {
class OptDialog;
}

class TextViewer;
class FreqDialog;
class QLineEdit;
class QSettings;
class QRegularExpressionValidator;

//---------------------------------------------------------------------------
class OptDialog : public QDialog
{
    Q_OBJECT

public:
    enum OptionsType {NaviOptions = 0, PostOptions = 1};

    explicit OptDialog(QWidget* parent, int opts);

    void loadOptions(QSettings &);
    void saveOptions(QSettings &);

    opt_t *appOptions;  // additional application specific options to load and save

    prcopt_t processingOptions;
    solopt_t solutionOptions;
    filopt_t fileOptions;

    // RTKNavi options
    int serverCycle, serverTolerance;
    int serverBufferSize, solutionBufferSize, navSelect, savedSolutions;
    int nmeaCycle, timeoutTime, reconnectTime;
    int monitorPort, fileSwapMargin, panelStacking;
    QString tlsSvrCertFile, tlsSvrKeyFile, tlsSvrCAFile, tlsSvrCADir;
    QString tlsCliCertFile, tlsCliKeyFile, tlsCliCAFile, tlsCliCADir;
    QString proxyAddress;
    QFont panelFont, positionFont;
    QColor panelFontColor, positionFontColor;

    // RTKPost options
    QString roverList, baseList;

protected:
    void showEvent(QShowEvent*);
    QString excludedSatellitesString(const prcopt_t *prcopt);
    bool fillExcludedSatellites(prcopt_t *prcopt, const QString &excludedSatellites);

    char proxyaddr[1024];  // proxy address stores in naviopts
    char tlssvrcertfile[1024], tlssvrkeyfile[1024], tlssvrcafile[1024], tlssvrcadir[1024];
    char tlsclicertfile[1024], tlsclikeyfile[1024], tlsclicafile[1024], tlsclicadir[1024];
    opt_t *naviopts;
    snrmask_t snrmask;
    int current_roverPositionType, current_referencePositionType;

    TextViewer *textViewer;
    FreqDialog * freqDialog;
    QRegularExpression regExDMSLat;
    QRegularExpression regExDMSLon;

protected slots:
    void accept();
    void selectAntennaPcvFile();
    void viewAntennaPcvFile();
    void loadSettings();
    void saveSettings();
    void selectReferencePosition();
    void selectRoverPosition();
    void viewStationPositionFile();
    void selectStationPositionFile();
    void referencePositionTypeChanged(int);
    void roverPositionTypeChanged(int);
    int getPosition(int type, QLineEdit **edit, double *pos);
    void setPosition(int type, QLineEdit **edit, const double *pos);
    void selectPanelFont();
    void selectSolutionFont();
    void selectGeoidDataFile();
    void viewSatelliteMetaFile();
    void selectSatelliteMetaFile();
    void viewSatellitePcvFile();
    void selectSatellitePcvFile();
    void selectLocalDirectory();
    void selectIonosphereFile();
    void selectDCBFile();
    void viewDCBFile();
    void selectEOPFile();
    void viewEOPFile();
    void selectBLQFile();
    void viewBLQFile();
    void selectElmaskFile();
    void viewElmaskFile();
    void selectTLSSvrCertFile();
    void viewTLSSvrCertFile();
    void selectTLSSvrKeyFile();
    void viewTLSSvrKeyFile();
    void selectTLSSvrCAFile();
    void viewTLSSvrCAFile();
    void selectTLSSvrCADir();
    void selectTLSCliCertFile();
    void viewTLSCliCertFile();
    void selectTLSCliKeyFile();
    void viewTLSCliKeyFile();
    void selectTLSCliCAFile();
    void viewTLSCliCAFile();
    void selectTLSCliCADir();
    void showSnrMaskDialog();
    void checkLineEditValidator();

private:
    void load(const QString &file);
    void save(const QString &file);
    void readAntennaList();
    void updateEnable();
    void showFrequenciesDialog();
    void showKeyDialog();
    void updateOptions();
    void updateUi(const prcopt_t &prcopt, const solopt_t &solopt, const filopt_t &filopt);
    QString stripped(const QString input, const QString suffix) const;
    int options;
    Ui::OptDialog *ui;

};
//---------------------------------------------------------------------------
#endif
