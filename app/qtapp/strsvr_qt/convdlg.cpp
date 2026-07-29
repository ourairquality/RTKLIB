//---------------------------------------------------------------------------
#include "convdlg.h"

#include "ui_convdlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
ConvDialog::ConvDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ConvDialog)
{
    ui->setupUi(this);

    for (int i = 0; i <= MAXRCVFMT; i++)
        ui->cBInputFormat->addItem(formatstrs[i]);
    ui->cBInputFormat->setCurrentIndex(1);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ConvDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ConvDialog::reject);
    connect(ui->cBConversion, &QCheckBox::toggled, this, &ConvDialog::updateEnable);
}
//---------------------------------------------------------------------------
void ConvDialog::updateEnable()
{
    ui->cBInputFormat->setEnabled(ui->cBConversion->isChecked());
    ui->cBOutputFormat->setEnabled(ui->cBConversion->isChecked());
    ui->lEOutputMessages->setEnabled(ui->cBConversion->isChecked());
    ui->lEOptions->setEnabled(ui->cBConversion->isChecked());
}
//---------------------------------------------------------------------------
void ConvDialog::setConversionEnabled(bool enable)
{
    ui->cBConversion->setChecked(enable);
    updateEnable();
}
//---------------------------------------------------------------------------
bool ConvDialog::getConversionEnabled()
{
    return ui->cBConversion->isChecked();
}
//---------------------------------------------------------------------------
void ConvDialog::setInputFormat(int format)
{
    ui->cBInputFormat->setCurrentIndex(format);
}
//---------------------------------------------------------------------------
int ConvDialog::getInputFormat()
{
    return ui->cBInputFormat->currentIndex();
}
//---------------------------------------------------------------------------
void ConvDialog::setOutputFormat(int format)
{
    ui->cBOutputFormat->setCurrentIndex(format);
}
//---------------------------------------------------------------------------
int ConvDialog::getOutputFormat()
{
    return ui->cBOutputFormat->currentIndex();
}
//---------------------------------------------------------------------------
void ConvDialog::setConversionMessage(const QString &msg)
{
    ui->lEOutputMessages->setText(msg);
}
//---------------------------------------------------------------------------
QString ConvDialog::getConversionMessage()
{
    return ui->lEOutputMessages->text();
}
//---------------------------------------------------------------------------
void ConvDialog::setConversionOptions(const QString &opt)
{
    ui->lEOptions->setText(opt);
}
//---------------------------------------------------------------------------
QString ConvDialog::getConversionOptions()
{
    return ui->lEOptions->text();
}
//---------------------------------------------------------------------------
