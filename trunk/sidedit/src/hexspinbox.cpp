#include <QtGui>

#include "hexspinbox.h"

HexSpinBox::HexSpinBox(QWidget *parent)
    : QSpinBox(parent)
{
    setRange(0, 255);
    validator = new QRegExpValidator(QRegExp("[0-9A-Fa-f]{1,8}"), this);
}

QValidator::State HexSpinBox::validate(QString &input, int &pos) const
{
    int  len  = input.size();
    int  from = 0;
    int  size = len;
    bool changed = false;

    QString prefix = this->prefix ();
    if (prefix.size() && input.startsWith(prefix))
    {
        from   += prefix.size ();
        size   -= from;
        changed = true;
    }

    QString suffix = this->suffix ();
    if (suffix.size() && input.endsWith(suffix))
    {
        size   -= suffix.size ();
        changed = true;
    }

    if (changed)
        input = input.mid (from, size);

    input = input.trimmed();
    pos  -= (len - input.size());

    QValidator::State state = validator->validate (input, pos);
    pos  += prefix.size ();
    input = prefix + input + suffix;

    return state;
}

QString HexSpinBox::textFromValue(int value) const
{
    QString max = QString::number (maximum(), 16);
    return QString::number(value, 16).toUpper().rightJustified (max.size(), '0');
}

int HexSpinBox::valueFromText(const QString &text) const
{
    bool ok;
    return text.toInt(&ok, 16);
}
