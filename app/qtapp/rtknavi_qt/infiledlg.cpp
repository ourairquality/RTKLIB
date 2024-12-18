//---------------------------------------------------------------------------

#include <QComboBox>
#include <QFileInfo>
#include <QShowEvent>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QCompleter>

#include "navimain.h"
#include "infiledlg.h"

#include "ui_infiledlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
InputFileDialog::InputFileDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::InputFileDialog)
{
    ui->setupUi(this);

    // setup completers
    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lEFilePath1->setCompleter(fileCompleter);
    ui->lEFilePath2->setCompleter(fileCompleter);
    ui->lEFilePath3->setCompleter(fileCompleter);
    ui->lEFilePath4->setCompleter(fileCompleter);
    ui->lEFilePath5->setCompleter(fileCompleter);
    ui->lEFilePath6->setCompleter(fileCompleter);

    // line edit actions
    QAction *aclEFilePath1Select = ui->lEFilePath1->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath1Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath2Select = ui->lEFilePath2->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath2Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath3Select = ui->lEFilePath3->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath3Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath4Select = ui->lEFilePath4->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath4Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath5Select = ui->lEFilePath5->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath5Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath6Select = ui->lEFilePath6->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath6Select->setToolTip(tr("Select File"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &InputFileDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &InputFileDialog::reject);

    connect(aclEFilePath1Select, &QAction::triggered, this, &InputFileDialog::selectFile1);
    connect(aclEFilePath2Select, &QAction::triggered, this, &InputFileDialog::selectFile2);
    connect(aclEFilePath3Select, &QAction::triggered, this, &InputFileDialog::selectFile3);
    connect(aclEFilePath4Select, &QAction::triggered, this, &InputFileDialog::selectFile4);
    connect(aclEFilePath5Select, &QAction::triggered, this, &InputFileDialog::selectFile5);
    connect(aclEFilePath6Select, &QAction::triggered, this, &InputFileDialog::selectFile6);
}
//---------------------------------------------------------------------------
void InputFileDialog::setPath(int n, const QString & path)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3,
      ui->lEFilePath4, ui->lEFilePath5, ui->lEFilePath6};

    if ((n < 0) || (n > 6)) return;

    edits[n]->setText(path);
}
//---------------------------------------------------------------------------
QString InputFileDialog::getPath(int n)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3,
      ui->lEFilePath4, ui->lEFilePath5, ui->lEFilePath6};
    if ((n < 0) || (n > 6)) return QString();
    return edits[n]->text();
}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile1()
{
  QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath1->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath1->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile2()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath2->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath2->setText(QDir::toNativeSeparators(dir));}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile3()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath3->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath3->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile4()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath4->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath4->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile5()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath5->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath5->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputFileDialog::selectFile6()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath6->text(),
                                             tr("All (*.*);;Precise Ephemeris (*.sp3 *.SP3);;Clock RINEX (*.clk *.CLK);;Earth Orientation Parameters (*.erp *.ERP)"));
    if (dir.isEmpty()) return;
    ui->lEFilePath6->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
