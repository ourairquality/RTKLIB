//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include <stdio.h>

#include <QProcess>
#include <QUrl>
#include <QDir>

#include "tcpoptdlg.h"
#include "mntpoptdlg.h"

#include "ui_tcpoptdlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------

#define NTRIP_TIMEOUT           10000                   // response timeout (ms)
#define NTRIP_CYCLE             50                      // processing cycle (ms)
#define NTRIP_MAXSTR            256                     // Max length of mountpoint string.
#define MAXSRCTBL               512000                  // max source table size (bytes)
#define ENDSRCTBL               "ENDSOURCETABLE"        // end marker of table
#define MAXLINE                 1024                    // max line size (byte)

//---------------------------------------------------------------------------
TcpOptDialog::TcpOptDialog(QWidget *parent, int options)
    : QDialog(parent), ui(new Ui::TcpOptDialog)
{
    ui->setupUi(this);

    mntpOptDialog = new MntpOptDialog(this);
    prevEnableTLS = 0;

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &TcpOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &TcpOptDialog::reject);
    connect(ui->btnNtrip, &QPushButton::clicked, this, &TcpOptDialog::btnNtripClicked);
    connect(ui->btnMountpointOptions, &QPushButton::clicked, this, &TcpOptDialog::btnMountpointClicked);
    connect(ui->btnBrowse, &QPushButton::clicked, this, &TcpOptDialog::btnBrowseClicked);
    connect(ui->cBEnableTLS, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TcpOptDialog::enableTLSChanged);

    setOptions(options);
}
void TcpOptDialog::updateEnable()
{
    unsigned enidx = ui->cBEnableTLS->currentIndex();
    ui->lblTLSVerify->setVisible(showOptions >= 0 && showOptions <= 5 && enidx > 0);
    ui->cBTLSVerify->setVisible(showOptions >= 0 && showOptions <= 5 && enidx > 0);
}
// callback on enable tls change --------------------------------------------
void TcpOptDialog::enableTLSChanged()
{
  if (ui->cBEnableTLS->currentIndex() > 0 && prevEnableTLS == 0) {
    // TLS is being changed to enabled.
    //
    // Default the Ntrip client version to 2 and the caster version to 0 (both).
    if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT)
      ui->cBNtripVer->setCurrentIndex(2);
    else if (showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT)
      ui->cBNtripVer->setCurrentIndex(0);
    // For a client default TLS Verify on.
    if (showOptions == OPT_TCP_CLIENT || showOptions == OPT_NTRIP_SOURCE ||
        showOptions == OPT_NTRIP_CLIENT)
      ui->cBTLSVerify->setCurrentIndex(1);
    // If the port is undefined then set to 443.
    QString port = ui->sBPort->text();
    if (port.isEmpty() || port == "0") ui->sBPort->setValue(443);
    else if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT
             || showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
      // If this a Ntrip connection and the port is 2201 then change to 443.
      if (port == "2201") ui->sBPort->setValue(443);
    }
  }
  prevEnableTLS = ui->cBEnableTLS->currentIndex();
  updateEnable();
}
//---------------------------------------------------------------------------
void TcpOptDialog::setOptions(int options)
{
    QString ti[] = {tr("TCP Server Options "),
                    tr("TCP Client Options"),
                    tr("NTRIP Source Options"),
                    tr("NTRIP Client Options"),
                    tr("NTRIP Caster Client Options"),
                    tr("NTRIP Caster Source Options"),
                    tr("UDP Server Options"),
                    tr("UDP Client Options")};

    ui->lblAddress->setText((options >= 2 && options <= 5) ? tr("NTRIP Caster Address") : tr("Server Address"));
    ui->lblMountPoint->setEnabled(options >= 2 && options <= 5);
    ui->cBMountPoint->setEnabled(options >= 2 && options <= 5);
    ui->lblUser->setEnabled(options >= 2 && options <= 5);
    ui->lEUser->setEnabled(options >= 2 && options <= 5);
    ui->lblPassword->setEnabled(options >= 2 && options <= 5);
    ui->lEPassword->setEnabled(options >= 2 && options <= 5);
    ui->btnNtrip->setVisible((options == 3));
    ui->btnBrowse->setVisible((options == 3));
    ui->btnMountpointOptions->setVisible((options >= 2 && options <= 5));
    ui->btnNtrip->setVisible(options == 2 || options == 3);

    ui->lblNtripVer->setVisible(options >= 2 && options <= 5);
    ui->cBNtripVer->setVisible(options >= 2 && options <= 5);
    ui->cBNtripVer->clear();
    if (options == 4 || options == 5) ui->cBNtripVer->addItem("Both 1 and 2");
    else ui->cBNtripVer->addItem("Default");
    ui->cBNtripVer->addItem("Only 1");
    ui->cBNtripVer->addItem("Only 2");

    ui->cBEnableTLS->setVisible(options >= 0 && options <= 5);
    ui->cBEnableTLS->clear();
    ui->cBEnableTLS->addItem("None");
    ui->cBEnableTLS->addItem("Require TLS");
    if (options >= 0 && options <= 5) {
      if (options == 4 || options == 5) {
        ui->cBEnableTLS->addItem("Allow TLS");
      } else {
        if (ui->cBEnableTLS->currentIndex() > 1)
          ui->cBEnableTLS->setCurrentIndex(1);
      }
    }
    setWindowTitle(ti[options]);

    showOptions = options;
    prevEnableTLS = ui->cBEnableTLS->currentIndex();
    updateEnable();
}

//---------------------------------------------------------------------------
void TcpOptDialog::setHistory(QString tcpHistory[], int size)
{
    ui->cBAddress->clear();

    for (int i = 0; i < qMin(size, MAXHIST); i++) {
        this->history[i] = tcpHistory[i];
        if (!history[i].isEmpty())
            ui->cBAddress->addItem(history[i]);
    }
}
//---------------------------------------------------------------------------
QString* TcpOptDialog::getHistory()
{
    return history;
}
//---------------------------------------------------------------------------
void TcpOptDialog::setPath(QString path)
{
    // Options.
    unsigned tls = 0, tlsVerify = 0, ntripVersion = 0;
    QStringList tokens = path.split("::");
    QString token;
    foreach(token, tokens.mid(1)) {
      if (showOptions == OPT_TCP_SERVER || showOptions == OPT_TCP_CLIENT ||
          showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT ||
          showOptions == OPT_NTRIP_CASTER_CLIENT || showOptions == OPT_NTRIP_CASTER_SOURCE) {
        if (token == 'S') tls = 1;
        if (token == 'A') {
          tls = 2;
          if (showOptions != OPT_NTRIP_CASTER_CLIENT && showOptions != OPT_NTRIP_CASTER_SOURCE)
            tls = 1;
        }
        if (token.contains('V')) {
          tlsVerify = QStringView{token}.mid(2).toUInt();
          if (tlsVerify > 1) tlsVerify = 1;
        }
      }
      if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT ||
          showOptions == OPT_NTRIP_CASTER_CLIENT || showOptions == OPT_NTRIP_CASTER_SOURCE) {
        if (token.contains('N')) ntripVersion = QStringView{token}.mid(2).toUInt();
        if (ntripVersion > 2) ntripVersion = 2;
      }
    }

    path = tokens.at(0);
    int userpwEnd = path.lastIndexOf("@");
    int addrStart = userpwEnd < 0 ? 0 : userpwEnd + 1;
    int mntpntStart = path.indexOf("/", addrStart);

    QString addrport;
    QString mntpnt;
    QString mntpntstr;
    if (mntpntStart >= 0) {
        int pathStart = mntpntStart + 1;
        if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT
            || showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
            int mntpntEnd = path.indexOf(":", pathStart);
            if (mntpntEnd >= pathStart) {
                mntpnt = path.mid(pathStart, mntpntEnd - pathStart);
                mntpntstr = path.mid(mntpntEnd + 1);
            } else {
                mntpnt = path.mid(pathStart);
            }
        } else {
            mntpnt = path.mid(pathStart);
        }
        char mntpntUnescaped[256];
        strurlunescape(qPrintable(mntpnt), 0, SIZE_MAX, mntpntUnescaped, sizeof(mntpntUnescaped));
        ui->cBMountPoint->setCurrentText(mntpnt);
        addrport = path.mid(addrStart, mntpntStart - addrStart);
    } else {
        addrport = path.mid(addrStart);
    }
    char mntpntUnescaped[256], mntpntstrUnescaped[NTRIP_MAXSTR];
    strurlunescape(qPrintable(mntpnt), 0, SIZE_MAX, mntpntUnescaped, sizeof(mntpntUnescaped));
    strurlunescape(qPrintable(mntpntstr), 0, SIZE_MAX, mntpntstrUnescaped, sizeof(mntpntstrUnescaped));
    QString mntpntUnescaped2 = mntpntUnescaped;
    QString mntpntstrUnescaped2 = mntpntstrUnescaped;
    ui->cBMountPoint->insertItem(0, mntpntUnescaped2, mntpntstrUnescaped2);
    ui->cBMountPoint->setCurrentText(mntpnt);

    QString user, password;
    if (userpwEnd >= 0) {
        QString userpasswd = path.mid(0, userpwEnd);
        int userEnd = userpasswd.indexOf(":");
        if (userEnd >= 0 ) {
            user = userpasswd.mid(0, userEnd);
            password = userpasswd.mid(userEnd + 1);
        } else {
            user = userpasswd;
        }
    }
    // Unescape path components.
    char userUnescaped[256], passwordUnescaped[256];
    strurlunescape(qPrintable(user), 0, SIZE_MAX, userUnescaped, sizeof(userUnescaped));
    strurlunescape(qPrintable(password), 0, SIZE_MAX, passwordUnescaped, sizeof(passwordUnescaped));
    ui->lEUser->setText(userUnescaped);
    ui->lEPassword->setText(passwordUnescaped);

    int port = 0;
    int portSep = addrport.indexOf(":");
    if (portSep >= 0)
        port = addrport.mid(portSep + 1).toInt();
    ui->sBPort->setValue(port);

    QString addr = addrport.mid(0, portSep);
    char addrUnescaped[256];
    strurlunescape(qPrintable(addr), 0, SIZE_MAX, addrUnescaped, sizeof(addrUnescaped));
    ui->cBAddress->insertItem(0, addrUnescaped);
    ui->cBAddress->setCurrentText(addrUnescaped);

    if (port == 443 && tls == 0) {
      // If port is 443 then force on TLS.
      tls = (showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) ? 2 : 1;
      // Default the Ntrip client version to 2 and the caster version to 0 (both).
      if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT)
        ntripVersion = 2;
      else if (showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT)
        ntripVersion = 0;
      // For a client default TLS Verify on.
      if (showOptions == OPT_TCP_CLIENT || showOptions == OPT_NTRIP_SOURCE ||
          showOptions == OPT_NTRIP_CLIENT)
        tlsVerify = 1;
    }

    ui->cBEnableTLS->setCurrentIndex(tls);
    ui->cBTLSVerify->setCurrentIndex(tlsVerify);
    ui->cBNtripVer->setCurrentIndex(ntripVersion);

    addHistory(ui->cBAddress, history);
    updateEnable();
}
//---------------------------------------------------------------------------
QString TcpOptDialog::getPath() {
    QString path;

    if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT
        || showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
      QString user = ui->lEUser->text();
      QString password = ui->lEPassword->text();
      if (!user.isEmpty() || !password.isEmpty()) {
        char userEscaped[256 * 3];
        strurlescape(qPrintable(user), 0, SIZE_MAX, userEscaped, sizeof(userEscaped));
        path = userEscaped;
        if (!password.isEmpty()) {
          char passwordEscaped[256 * 3];
          strurlescape(qPrintable(password), 0, SIZE_MAX, passwordEscaped, sizeof(passwordEscaped));
          path = QStringLiteral("%1:%2").arg(path, passwordEscaped);
        }
        if (!path.isEmpty()) path += "@";
      }
    }
    char addrEscaped[256 * 3];
    strurlescape(qPrintable(ui->cBAddress->currentText()), 0, SIZE_MAX, addrEscaped, sizeof(addrEscaped));
    path = QStringLiteral("%1%2").arg(path, addrEscaped);

    QString port = ui->sBPort->text();
    if (!port.isEmpty()) {
      char portEscaped[256 * 3];
      strurlescape(qPrintable(port), 0, SIZE_MAX, portEscaped, sizeof(portEscaped));
      path = QStringLiteral("%1:%2").arg(path, portEscaped);
    }
    if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT ||
        showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
      QString mntpnt = ui->cBMountPoint->currentText();
      QString str = ui->cBMountPoint->currentData().toString();
      if (!mntpnt.isEmpty() || !str.isEmpty()) {
        char mntpntEscaped[256 * 3];
        strurlescape(qPrintable(mntpnt), 0, SIZE_MAX, mntpntEscaped, sizeof(mntpntEscaped));
        path = QStringLiteral("%1/%2").arg(path, mntpntEscaped);
        if (!str.isEmpty()) {
          char strEscaped[256 * 3];
          strurlescape(qPrintable(str), 0, SIZE_MAX, strEscaped, sizeof(strEscaped));
          path = QStringLiteral("%1:%2").arg(path, strEscaped);
        }
      }
    }
    if (showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT ||
        showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
      if (ui->cBNtripVer->currentIndex() > 0)
        path = path + (ui->cBNtripVer->currentIndex() > 1 ? "::N=2" : "::N=1");
    }
    if (showOptions == OPT_NTRIP_CASTER_CLIENT) path = path + "::T=1";
    else if (showOptions == OPT_NTRIP_CASTER_SOURCE) path = path + "::T=2";
    if (showOptions == OPT_TCP_CLIENT || showOptions == OPT_TCP_SERVER || showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT) {
      if (ui->cBEnableTLS->currentIndex() > 0) path = path + "::S";
    } else if (showOptions == OPT_NTRIP_CASTER_SOURCE || showOptions == OPT_NTRIP_CASTER_CLIENT) {
      if (ui->cBEnableTLS->currentIndex() == 1)
        path = path + "::S";
      else if (ui->cBEnableTLS->currentIndex() == 2)
        path = path + "::A";
    }
    if (showOptions == OPT_TCP_SERVER || showOptions == OPT_TCP_CLIENT ||
        showOptions == OPT_NTRIP_SOURCE || showOptions == OPT_NTRIP_CLIENT ||
        showOptions == OPT_NTRIP_CASTER_CLIENT || showOptions == OPT_NTRIP_CASTER_SOURCE) {
      path = path + (ui->cBTLSVerify->currentIndex() > 0 ? "::V=1" : "::V=0");
    }
    return path;
}

//---------------------------------------------------------------------------
void TcpOptDialog::accept()
{
    addHistory(ui->cBAddress, history);

    QDialog::accept();
}
//---------------------------------------------------------------------------
void TcpOptDialog::addHistory(QComboBox *list, QString *hist)
{
    for (int i = 0; i < MAXHIST; i++) {
        if (list->currentText() != hist[i]) continue;
        for (int j = i + 1; j < MAXHIST; j++)
            hist[j - 1] = hist[j];
        hist[MAXHIST - 1] = "";
	}
    for (int i = MAXHIST - 1; i > 0; i--)
        hist[i] = hist[i - 1];
    hist[0] = list->currentText();

    list->clear();
    for (int i = 0; i < MAXHIST; i++)
        if (!hist[i].isEmpty()) list->addItem(hist[i]);
    list->setCurrentIndex(0);
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnNtripClicked()
{
    QPushButton *btn = (QPushButton *)sender();
    QString path = ui->cBAddress->currentText() + ":" + ui->sBPort->text();
    stream_t str;
    uint32_t tick = tickget();

    if (ui->cBNtripVer->currentIndex() > 0)
      path = path + (ui->cBNtripVer->currentIndex() > 1 ? "::N=2" : "::N=1");
    if (ui->cBEnableTLS->currentIndex() > 0) path = path + "::S";
    path = path + (ui->cBTLSVerify->currentIndex() > 0 ? "::V=1" : "::V=0");

    strinit(&str);
    if (!stropen(&str, STR_NTRIPCLI, STR_MODE_R, qPrintable(path))) return;

    char *buff = (char *)malloc(MAXSRCTBL);
    if (buff == NULL) {
      strclose(&str);
      return;
    }

    btn->setEnabled(false);
    buff[0] = '\0';
    size_t pi = 0;
    while (pi + 1 < MAXSRCTBL) {
        pi += strread(&str, (uint8_t *)buff, MAXSRCTBL, pi, MAXSRCTBL - pi - 1);
        buff[pi] = '\0';
        sleepms(NTRIP_CYCLE);
        if (strstr(buff, ENDSRCTBL)) break;
        if ((int)(tickget() - tick) > NTRIP_TIMEOUT) break;
        if (strstat(&str, NULL, 0) <= 0) break;
    }
    strclose(&str);

    ui->cBMountPoint->clear();
    for (char *p = buff; (p  = strstr(p, "STR;")); p+=4) {
        char mntpnt[256];
        if (sscanf(p, "STR;%255[^;]", mntpnt) == 1) {
            QString str = QString(p).split('\n', Qt::SkipEmptyParts).first().trimmed();
            ui->cBMountPoint->addItem(mntpnt, str);
        }
    }
    free(buff);
    btn->setEnabled(true);
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnBrowseClicked()
{
    QStringList cmds = {"srctblbrows_qt", "../../../bin/srctblbrows_qt", "../srctblbrows_qt/srctblbrows_qt"};
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QString addrText = ui->cBAddress->currentText();
    QString portText = ui->sBPort->text();

    if (!portText.isEmpty()) addrText += ":" + portText;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), QStringList(addrText), 1)) {
            return;
        }
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnMountpointClicked()
{
    mntpOptDialog->setMountPoint(ui->cBMountPoint->currentText());
    mntpOptDialog->setMountPointString(ui->cBMountPoint->currentData().toString());

    mntpOptDialog->exec();
    if (mntpOptDialog->result()!=QDialog::Accepted) return;

    ui->cBMountPoint->setCurrentText(mntpOptDialog->getMountPoint());
    ui->cBMountPoint->setItemData(ui->cBMountPoint->currentIndex(), mntpOptDialog->getMountPointString());
}
//---------------------------------------------------------------------------
int TcpOptDialog::execCommand(const QString &cmd, const QStringList &opt, int show)
{
    Q_UNUSED(show);

    return QProcess::startDetached(cmd, opt);
}
//---------------------------------------------------------------------------
