//---------------------------------------------------------------------------

#ifndef plotoptH
#define plotoptH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QSettings>

#define MAXMAPLAYER 12                  // max number of map layers

namespace Ui {
    class PlotOptDialog;
}
//class Plot;
class RefDialog;

//---------------------------------------------------------------------------
class PlotOptDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PlotOptDialog(QWidget *parent=NULL);
    void loadOptions(QSettings &);
    void saveOptions(QSettings &);

    // plot options
    int getTimeFormat();
    int getLatLonFormat();
    int getShowStats();
    int getShowSlip();
    int getShowHalfC();
    int getShowEphemeris();
    double getElevationMask();
    int getElevationMaskEnabled();
    int getHideLowSatellites();
    double getMaxDop();
    double getMaxMP();
    int getNavSys();
    QString getExcludedSatellites();
    int getShowError();
    int getShowArrow();
    int getShowGridLabel();
    int getShowLabel();
    int getShowCompass();
    int getShowScale();
    int getAutoScale();
    double getYRange();
    int getRtBufferSize();
    int getTimeSyncOut();
    int getTimeSyncPort();
    int getOrigin();
    int getReceiverPosition();
    double* getOoPosition();
    int getPlotStyle();
    int getMarkSize();
    int getAnimationCycle();
    int getRefreshCycle();
    QString getShapeFile();
    QString getTleFile();
    QString getRinexOptions();
    QString getTleSatelliteFile();
    QString getSigDef();
    QFont getFont();
    QColor getCColor(int i);
    QColor getMarkerColor(int i, int j);

protected slots:
    void accept();
    void color1Select();
    void color2Select();
    void color3Select();
    void color4Select();
    void referencePositionSelect();
    void markerColorSelect();
    void fontSelect();
    void tleFileOpen();
    void shapeFileOpen();
    void tleSatelliteFileOpen();
    void tleFileView();
    void tleSatelliteFileView();

private:
    void updateFont();
    void updateEnable();
    void updateConfig();

protected:
    QColor markerColor[2][8]; // {{mark1 0-7},{mark2 0-7}}
    QColor cColor[4];    // {background,grid,text,line}

    QFont fontOption;
    RefDialog *refDialog;

    double elevation_mask;
    int timeFormat, elevationMaskEnabled;

private:
    Ui::PlotOptDialog *ui;
};
//---------------------------------------------------------------------------
#endif
