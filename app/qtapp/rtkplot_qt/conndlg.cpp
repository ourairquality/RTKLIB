//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QCompleter>
#include <QFileDialog>
#include <QAction>

#include "serioptdlg.h"
#include "fileoptdlg.h"
#include "tcpoptdlg.h"
#include "cmdoptdlg.h"
#include "conndlg.h"

#include "ui_conndlg.h"

#include "rtklib.h"


//---------------------------------------------------------------------------

ConnectDialog::ConnectDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ConnectDialog)
{
    ui->setupUi(this);

    for (int i = 0; i < 2; i++)
        for (int j  = 0; j < 2; j++)
            commandEnable[i][j] = 0;

    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lETLSSvrCertFile->setCompleter(fileCompleter);
    ui->lETLSSvrKeyFile->setCompleter(fileCompleter);
    ui->lETLSSvrCAFile->setCompleter(fileCompleter);
    ui->lETLSSvrCADir->setCompleter(fileCompleter);
    ui->lETLSCliCertFile->setCompleter(fileCompleter);
    ui->lETLSCliKeyFile->setCompleter(fileCompleter);
    ui->lETLSCliCAFile->setCompleter(fileCompleter);
    ui->lETLSCliCADir->setCompleter(fileCompleter);

    QAction *acTLSSvrCertFile = ui->lETLSSvrCertFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSSvrCertFile->setToolTip(tr("Select TLS server certificate file"));

    QAction *acTLSSvrKeyFile = ui->lETLSSvrKeyFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSSvrKeyFile->setToolTip(tr("Select TLS server certificate private key file"));

    QAction *acTLSSvrCAFile = ui->lETLSSvrCAFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSSvrCAFile->setToolTip(tr("Select TLS server CA file"));

    QAction *acTLSSvrCADir = ui->lETLSSvrCADir->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSSvrCADir->setToolTip(tr("Select TLS server CA directory"));

    QAction *acTLSCliCertFile = ui->lETLSCliCertFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSCliCertFile->setToolTip(tr("Select TLS client certificate file"));

    QAction *acTLSCliKeyFile = ui->lETLSCliKeyFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSCliKeyFile->setToolTip(tr("Select TLS client certificate private key file"));

    QAction *acTLSCliCAFile = ui->lETLSCliCAFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSCliCAFile->setToolTip(tr("Select TLS client CA file"));

    QAction *acTLSCliCADir = ui->lETLSCliCADir->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLSCliCADir->setToolTip(tr("Select TLS client CA directory"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ConnectDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ConnectDialog::reject);
    connect(ui->btnCommand1, &QPushButton::clicked, this, &ConnectDialog::selectCommandsStream1);
    connect(ui->btnCommand2, &QPushButton::clicked, this, &ConnectDialog::selectCommandsStream2);
    connect(ui->btnOption1, &QPushButton::clicked, this, &ConnectDialog::selectOptionsStream1);
    connect(ui->btnOption2, &QPushButton::clicked, this, &ConnectDialog::selectOptionsStream2);
    connect(ui->cBSolutionFormat1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ConnectDialog::updateEnable);
    connect(ui->cBSolutionFormat2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ConnectDialog::updateEnable);
    connect(ui->cBSelectStream1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ConnectDialog::updateEnable);
    connect(ui->cBSelectStream2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ConnectDialog::updateEnable);

    connect(acTLSSvrCertFile, &QAction::triggered, this, &ConnectDialog::tlsSvrCertFileSelect);
    connect(acTLSSvrKeyFile, &QAction::triggered, this, &ConnectDialog::tlsSvrKeyFileSelect);
    connect(acTLSSvrCAFile, &QAction::triggered, this, &ConnectDialog::tlsSvrCAFileSelect);
    connect(acTLSSvrCADir, &QAction::triggered, this, &ConnectDialog::tlsSvrCADirSelect);
    connect(acTLSCliCertFile, &QAction::triggered, this, &ConnectDialog::tlsCliCertFileSelect);
    connect(acTLSCliKeyFile, &QAction::triggered, this, &ConnectDialog::tlsCliKeyFileSelect);
    connect(acTLSCliCAFile, &QAction::triggered, this, &ConnectDialog::tlsCliCAFileSelect);
    connect(acTLSCliCADir, &QAction::triggered, this, &ConnectDialog::tlsCliCADirSelect);
}
//---------------------------------------------------------------------------
void ConnectDialog::selectOptionsStream1()
{
    switch (ui->cBSelectStream1->currentIndex()) {
      case 1: serialOptionsStream(0, 0, 0); break;
      case 2: tcpOption(0, TcpOptDialog::OPT_TCP_CLIENT, 1);   break;
      case 3: tcpOption(0, TcpOptDialog::OPT_TCP_SERVER, 2);   break;
      case 4: tcpOption(0, TcpOptDialog::OPT_NTRIP_CLIENT, 3);   break;
      case 5: tcpOption(0, TcpOptDialog::OPT_NTRIP_CASTER_SOURCE, 4); break;
      case 6: tcpOption(0, TcpOptDialog::OPT_UDP_SERVER, 5); break;
      case 7: fileOption(0, 0, 6);   break;
    }
}
//---------------------------------------------------------------------------
void ConnectDialog::selectOptionsStream2()
{
    switch (ui->cBSelectStream2->currentIndex()) {
      case 1: serialOptionsStream(1, 0, 0); break;
      case 2: tcpOption(1, TcpOptDialog::OPT_TCP_CLIENT, 1);   break;
      case 3: tcpOption(1, TcpOptDialog::OPT_TCP_SERVER, 2);   break;
      case 4: tcpOption(1, TcpOptDialog::OPT_NTRIP_CLIENT, 3);   break;
      case 5: tcpOption(1, TcpOptDialog::OPT_NTRIP_CASTER_SOURCE, 4); break;
      case 6: tcpOption(1, TcpOptDialog::OPT_UDP_SERVER, 5); break;
      case 7: fileOption(1, 0, 6);   break;
    }
}
//---------------------------------------------------------------------------
void ConnectDialog::selectCommandsStream1()
{
    CmdOptDialog dialog(this);

    for (int i = 0; i < 2; i++) {
        dialog.setCommands(i, commands[0][i]);
        dialog.setCommandsEnabled(i, commandEnable[0][i]);
    }
    dialog.exec();

    if (dialog.result() != QDialog::Accepted) return;

    for (int i = 0; i < 2; i++) {
        commands[0][i] = dialog.getCommands(i);
        commandEnable[0][i] = dialog.getCommandsEnabled(i);
    }
}
//---------------------------------------------------------------------------
void ConnectDialog::selectCommandsStream2()
{
    CmdOptDialog dialog(this);

    for (int i = 0; i < 2; i++) {
        dialog.setCommands(i, commands[1][i]);
        dialog.setCommandsEnabled(i, commandEnable[1][i]);
    }
    dialog.exec();

    if (dialog.result() != QDialog::Accepted) return;

    for (int i = 0; i < 2; i++) {
        commands[1][i] = dialog.getCommands(i);
        commandEnable[1][i] = dialog.getCommandsEnabled(i);
    }
}
//---------------------------------------------------------------------------
void ConnectDialog::serialOptionsStream(int stream,int opt,unsigned i)
{
    SerialOptDialog dialog(this);

    dialog.setPath(paths[stream][i]);
    dialog.setOptions(opt);
    dialog.exec();

    if (dialog.result() != QDialog::Accepted) return;

    paths[stream][i] = dialog.getPath();
}
//---------------------------------------------------------------------------
void ConnectDialog::tcpOption(int stream, int opt,unsigned i)
{
    // Initialize the TLS certificates.
    strinittls(qPrintable(ui->lETLSSvrCertFile->text()), qPrintable(ui->lETLSSvrKeyFile->text()),
               qPrintable(ui->lETLSSvrCAFile->text()), qPrintable(ui->lETLSSvrCADir->text()),
               0,
               qPrintable(ui->lETLSCliCertFile->text()), qPrintable(ui->lETLSCliKeyFile->text()),
               qPrintable(ui->lETLSCliCAFile->text()), qPrintable(ui->lETLSCliCADir->text()),
               0);

    TcpOptDialog dialog(this);

    dialog.setOptions(opt);
    dialog.setHistory(history, MAXHIST);
    dialog.setPath(paths[stream][i]);

    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;

    paths[stream][i] = dialog.getPath();
    for (int i = 0; i < MAXHIST; i++)
        history[i] = dialog.getHistory()[i];
}
//---------------------------------------------------------------------------
void ConnectDialog::fileOption(int stream, int opt,unsigned i)
{
    FileOptDialog dialog(this);

    dialog.setPath(paths[stream][i]);
    dialog.setOptions(opt);

    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;

    paths[stream][i] = dialog.getPath();
}
//---------------------------------------------------------------------------
void ConnectDialog::updateEnable()
{
    ui->btnOption1->setEnabled(ui->cBSelectStream1->currentIndex() > 0);
    ui->btnOption2->setEnabled(ui->cBSelectStream2->currentIndex() > 0);
    ui->btnCommand1->setEnabled(ui->cBSelectStream1->currentIndex() == 1);
    ui->btnCommand2->setEnabled(ui->cBSelectStream2->currentIndex() == 1);
    ui->cBSolutionFormat1->setEnabled(ui->cBSelectStream1->currentIndex() > 0);
    ui->cBSolutionFormat2->setEnabled(ui->cBSelectStream2->currentIndex() > 0);
    ui->cBTimeFormat->setEnabled(ui->cBSolutionFormat1->currentIndex() != 3 || ui->cBSolutionFormat2->currentIndex() != 3);
    ui->cBDegFormat->setEnabled(ui->cBSolutionFormat1->currentIndex() == 0 || ui->cBSolutionFormat2->currentIndex() == 0);
    ui->lEFieldSeperator->setEnabled(ui->cBSolutionFormat1->currentIndex() != 3 || ui->cBSolutionFormat2->currentIndex() != 3);
    ui->lblTimeFormat->setEnabled(ui->cBSolutionFormat1->currentIndex() != 3 || ui->cBSolutionFormat2->currentIndex() != 3);
    ui->lblLatLonFormat->setEnabled(ui->cBSolutionFormat1->currentIndex() == 0 || ui->cBSolutionFormat2->currentIndex() == 0);
    ui->lblFieldSeparator->setEnabled(ui->cBSolutionFormat1->currentIndex() != 3 || ui->cBSolutionFormat2->currentIndex() != 3);
    ui->lblTimeout->setEnabled((ui->cBSelectStream1->currentIndex() == 2 || ui->cBSelectStream1->currentIndex() == 4) ||
                               (ui->cBSelectStream2->currentIndex() == 2 || ui->cBSelectStream2->currentIndex() == 4));
    ui->lblReconnect->setEnabled((ui->cBSelectStream1->currentIndex() == 2 || ui->cBSelectStream1->currentIndex() == 4) ||
                                 (ui->cBSelectStream2->currentIndex() == 2 || ui->cBSelectStream2->currentIndex() == 4));
    ui->sBTimeoutTime->setEnabled((ui->cBSelectStream1->currentIndex() == 2 || ui->cBSelectStream1->currentIndex() == 4) ||
                                  (ui->cBSelectStream2->currentIndex() == 2 || ui->cBSelectStream2->currentIndex() == 4));
    ui->sBReconnectTime->setEnabled((ui->cBSelectStream1->currentIndex() == 2 || ui->cBSelectStream1->currentIndex() == 4) ||
                                    (ui->cBSelectStream2->currentIndex() == 2 || ui->cBSelectStream2->currentIndex() == 4));
}
//---------------------------------------------------------------------------
void ConnectDialog::setStreamType(int stream, int type)
{
    int str[] = {STR_NONE, STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_NTRIPCAS, STR_UDPSVR, STR_FILE};
    QComboBox *cBType[] = {ui->cBSelectStream1, ui->cBSelectStream2};

    for (int i = 0; i < 7; i++) {
        if (str[i] == type) cBType[stream]->setCurrentIndex(i);
    }
    updateEnable();
}
//---------------------------------------------------------------------------
int ConnectDialog::getStreamType(int stream)
{
    int str[] = {STR_NONE, STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_NTRIPCAS, STR_UDPSVR, STR_FILE};
    QComboBox *cBType[] = {ui->cBSelectStream1, ui->cBSelectStream2};

    return str[cBType[stream]->currentIndex()];
}
//---------------------------------------------------------------------------
void ConnectDialog::setStreamFormat(int stream, int format)
{
    QComboBox *cBFormat[] = {ui->cBSolutionFormat1, ui->cBSolutionFormat2};
    cBFormat[stream]->setCurrentIndex(format);

    updateEnable();
}
//---------------------------------------------------------------------------
int ConnectDialog::getStreamFormat(int stream)
{
    QComboBox *cBFormat[] = {ui->cBSolutionFormat1, ui->cBSolutionFormat2};
    return cBFormat[stream]->currentIndex();
}
//---------------------------------------------------------------------------
void ConnectDialog::setCommands(int stream, int type, const QString & cmd)
{
    commands[stream][type]= cmd;
}
//---------------------------------------------------------------------------
QString ConnectDialog::getCommands(int stream, int type)
{
    return commands[stream][type];
}
//---------------------------------------------------------------------------
void ConnectDialog::setCommandsEnabled(int stream, int type, bool ena)
{
    commandEnable[stream][type] = ena;
}
//---------------------------------------------------------------------------
bool ConnectDialog::getCommandsEnabled(int stream, int type)
{
    return commandEnable[stream][type];
}
//---------------------------------------------------------------------------
void ConnectDialog::setTimeFormat(int timeFormat)
{
    ui->cBTimeFormat->setCurrentIndex(timeFormat);
}
//---------------------------------------------------------------------------
int ConnectDialog::getTimeFormat()
{
    return ui->cBTimeFormat->currentIndex();
}
//---------------------------------------------------------------------------
void ConnectDialog::setDegFormat(int degFormat)
{
    ui->cBDegFormat->setCurrentIndex(degFormat);
}
//---------------------------------------------------------------------------
int ConnectDialog::getDegFormat()
{
    return ui->cBDegFormat->currentIndex();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTimeoutTime(int timeoutTime)
{
    ui->sBTimeoutTime->setValue(timeoutTime);
}
//---------------------------------------------------------------------------
int ConnectDialog::getTimeoutTime()
{
    return ui->sBTimeoutTime->value();
}
//---------------------------------------------------------------------------
void ConnectDialog::setReconnectTime(int reconnectTime)
{
    ui->sBReconnectTime->setValue(reconnectTime);
}
//---------------------------------------------------------------------------
int ConnectDialog::getReconnectTime()
{
    return ui->sBReconnectTime->value();
}
//---------------------------------------------------------------------------
void ConnectDialog::setFieldSeparator(const QString &fieldSeparator)
{
    ui->lEFieldSeperator->setText(fieldSeparator);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getFieldSeparator()
{
    return ui->lEFieldSeperator->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setPath(int stream, int type, const QString &path)
{
    paths[stream][type] = path;
}
//---------------------------------------------------------------------------
QString ConnectDialog::getPath(int stream, int type)
{
    return paths[stream][type];
}
//---------------------------------------------------------------------------
void ConnectDialog::setHistory(int i, const QString &history)
{
    if (i < MAXHIST)
        this->history[i] = history;
}
//---------------------------------------------------------------------------
const QString &ConnectDialog::getHistory(int i)
{
    return history[i];
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSSvrCertFile(QString file)
{
    ui->lETLSSvrCertFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSSvrCertFile()
{
    return ui->lETLSSvrCertFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSSvrKeyFile(QString file)
{
    ui->lETLSSvrKeyFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSSvrKeyFile()
{
    return ui->lETLSSvrKeyFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSSvrCAFile(QString file)
{
    ui->lETLSSvrCAFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSSvrCAFile()
{
    return ui->lETLSSvrCAFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSSvrCADir(QString file)
{
    ui->lETLSSvrCADir->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSSvrCADir()
{
    return ui->lETLSSvrCADir->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSCliCertFile(QString file)
{
    ui->lETLSCliCertFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSCliCertFile()
{
    return ui->lETLSCliCertFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSCliKeyFile(QString file)
{
    ui->lETLSCliKeyFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSCliKeyFile()
{
    return ui->lETLSCliKeyFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSCliCAFile(QString file)
{
    ui->lETLSCliCAFile->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSCliCAFile()
{
    return ui->lETLSCliCAFile->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::setTLSCliCADir(QString file)
{
    ui->lETLSCliCADir->setText(file);
}
//---------------------------------------------------------------------------
QString ConnectDialog::getTLSCliCADir()
{
    return ui->lETLSCliCADir->text();
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsSvrCertFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Server Certificate File"), ui->lETLSSvrCertFile->text(), tr("(*.crt *.cer *.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSSvrCertFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsSvrKeyFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Server Private Key File"), ui->lETLSSvrKeyFile->text(), tr("(*.key *.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSSvrKeyFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsSvrCAFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Server CA File"), ui->lETLSSvrCAFile->text(), tr("TLS Server CA File (*.crt *.cer *.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSSvrCAFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsSvrCADirSelect()
{
    QString dir = ui->lETLSSvrCADir->text();
    dir = QFileDialog::getExistingDirectory(this, tr("TLS Server CA Directory"), dir);
    if (!dir.isEmpty())
      ui->lETLSSvrCADir->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsCliCertFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Client Certificate File"), ui->lETLSCliCertFile->text(), tr("(*.crt *.cer */=.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSCliCertFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsCliKeyFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Client Private Key File"), ui->lETLSCliKeyFile->text(), tr("(*.key *.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSCliKeyFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsCliCAFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("TLS Client CA File"), ui->lETLSCliCAFile->text(), tr("TLS Server CA File (*.crt *.cer *.pem);;All (*.*)"));
    if (!filename.isEmpty())
      ui->lETLSCliCAFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void ConnectDialog::tlsCliCADirSelect()
{
    QString dir = ui->lETLSCliCADir->text();
    dir = QFileDialog::getExistingDirectory(this, tr("TLS Client CA Directory"), dir);
    if (!dir.isEmpty())
      ui->lETLSCliCADir->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
