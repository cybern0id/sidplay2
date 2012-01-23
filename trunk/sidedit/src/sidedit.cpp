#include <QCloseEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QTextDecoder>

#include "sidedit.h"
#include "ui_sidedit.h"

template <class T> inline T* SidEdit::_find (const char *objectName)
{
#define QT_WIDGET_NOT_FOUND obj
    T* obj = qFindChild<T*>(this, objectName);
    Q_ASSERT (QT_WIDGET_NOT_FOUND);
    return obj;
}

#ifndef QT_NO_DEBUG
static bool safe_connect (bool condition, int line)
{
    if (!condition)
        qt_assert("QT_CONNECTION_FAILED", __FILE__, line);
    return condition;
}
#define connect(...) safe_connect(connect (__VA_ARGS__),__LINE__)
#else
#define connect(...) connect (__VA_ARGS__)
#endif


SidEdit::PlayerType::PlayerType ()
{
    _add (SIDTUNE_COMPATIBILITY_C64,   QT_TR_NOOP("C64 Compatible"));
    _add (SIDTUNE_COMPATIBILITY_R64,   QT_TR_NOOP("C64 Only"));
    _add (SIDTUNE_COMPATIBILITY_BASIC, QT_TR_NOOP("C64 Basic"));
    _add (SIDTUNE_COMPATIBILITY_PSID,  QT_TR_NOOP("PlaySID Only"));
//    _add (MUSPLAYER, QT_TR_NOOP("Compute's Sidplayer"));
}

SidEdit::SIDTypeInfo::SIDTypeInfo ()
{
    _add (SIDTUNE_SIDMODEL_UNKNOWN, QT_TR_NOOP("<unknown>"));
    _add (SIDTUNE_SIDMODEL_6581,    QT_TR_NOOP("6581"));
    _add (SIDTUNE_SIDMODEL_8580,    QT_TR_NOOP("8580"));
    _add (SIDTUNE_SIDMODEL_ANY,     QT_TR_NOOP("Any"));
}

SidEdit::TimingInfo::TimingInfo ()
{
    _add (SIDTUNE_SPEED_VBI,    QT_TR_NOOP("VIC (Vertical Blank Interrupt)"));
    _add (SIDTUNE_SPEED_CIA_1A, QT_TR_NOOP("CIA (Default 60 Hz)"));
}

SidEdit::VICSpeedInfo::VICSpeedInfo ()
{
    _add (SIDTUNE_CLOCK_UNKNOWN, QT_TR_NOOP("<unknown>"));
    _add (SIDTUNE_CLOCK_PAL,     QT_TR_NOOP("50 Hz PAL (6569)"));
    _add (SIDTUNE_CLOCK_NTSC,    QT_TR_NOOP("60 Hz NTSC (6567R8)"));
    _add (SIDTUNE_CLOCK_ANY,     QT_TR_NOOP("Any"));
}

SidEdit::SidEdit(QWidget *parent, Qt::WFlags flags)
:QMainWindow(parent, flags)
,m_isModified(false)
,m_pages(0)
,m_tune(new SidTuneWrite(0))
{
    {   // Do this to ensure we are compatible with dynamic loading
        Ui::sideditClass ui;
        ui.setupUi(this);
    }

    m_cboPlayerType     = _find<QComboBox>("cboPlayerType");
    m_cboSidType        = _find<QComboBox>("cboSidType");
    m_cboSid2Type       = _find<QComboBox>("cboSid2Type");
    m_cboTiming         = _find<QComboBox>("cboTiming");
    m_cboVicSpeed       = _find<QComboBox>("cboVicSpeed");
    m_txtCreditTitle    = _find<QLineEdit>("txtCreditTitle");
    m_txtCreditAuthor   = _find<QLineEdit>("txtCreditAuthor");
    m_txtCreditReleased = _find<QLineEdit>("txtCreditReleased");
    m_txtLoadRange      = _find<QLabel>("txtLoadRange");
    m_sbrSong           = _find<QScrollBar>("sbrSong");
    m_spnInitAddress    = _find<HexSpinBox>("spnInitAddress");
    m_spnLoadAddress    = _find<HexSpinBox>("spnLoadAddress");
    m_spnPlayAddress    = _find<HexSpinBox>("spnPlayAddress");
    m_spnPlayerStart    = _find<HexSpinBox>("spnPlayerStart");
    m_spnPlayerEnd      = _find<HexSpinBox>("spnPlayerEnd");
    m_spnSid2Address    = _find<HexSpinBox>("spnSid2Address");
    m_spnSongs          = _find<QSpinBox>("spnSongs");
    m_txtPlayerPages    = _find<QLabel>("txtPlayerPages");
    m_txtSong           = _find<QLabel>("txtSong");

    connect (m_spnPlayerEnd,   SIGNAL(valueChanged(int)),        SLOT (_playerEndChanged(int)));
    connect (m_spnPlayerStart, SIGNAL(valueChanged(int)),        SLOT (_playerStartChanged(int)));
    connect (m_cboPlayerType,  SIGNAL(currentIndexChanged(int)), SLOT (_playerTypeChanged(int)));

    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditTitle"),    SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditAuthor"),   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditReleased"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditTitle,    SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditAuthor,   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditReleased, SLOT(setEnabled(bool)));

    connect (m_spnSongs, SIGNAL(valueChanged(int)), this, SLOT(_songsChanged(int)));
    connect (m_sbrSong,  SIGNAL(valueChanged(int)), this, SLOT(_songChanged(int)));

    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblSidType"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_cboSidType,  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblSid2Type"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_cboSid2Type,  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblSid2Address"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_spnSid2Address,  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblVicSpeed"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_cboVicSpeed, SLOT(setEnabled(bool)));

    connect (_find<QObject>("btnDefaultSong"), SIGNAL(clicked()), this, SLOT(_defaultClicked()));
    connect (this, SIGNAL(_defaultSong(int)), _find<QObject>("stkDefaultSong"), SLOT(setCurrentIndex(int)));

    connect (this, SIGNAL(_timingEnabled(bool)), _find<QObject>("lblTiming"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_timingEnabled(bool)), m_cboTiming, SLOT(setEnabled(bool)));

    _updateComboBox (m_cboPlayerType, m_playerType);
    _updateComboBox (m_cboSidType,    m_sidTypeInfo);
    _updateComboBox (m_cboSid2Type,   m_sidTypeInfo);
    _updateComboBox (m_cboTiming,     m_timingInfo);
    _updateComboBox (m_cboVicSpeed,   m_vicSpeedInfo);

    _new ();
}

SidEdit::~SidEdit ()
{
    ;
}

void SidEdit::closeEvent (QCloseEvent *event)
{
    if (_maybeSave())
    {
        event->accept();
        return;
    }
    event->ignore();
}

void SidEdit::_defaultClicked ()
{
    m_defaultSong = m_sbrSong->value ();
    _defaultSong (true);
}

void SidEdit::_loadFile (const QString &fileName)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    std::auto_ptr<SidTuneWrite> tune;

    try
    {
        QFileInfo info(fileName);
        tune.reset (new SidTuneWrite(fileName.toUtf8()));

/*
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::critical(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(fileName)
                             .arg(file.errorString()));
        return;
    }
*/
        if (!tune->getStatus())
            tune.reset (0);
        if (!tune->selectSong(0))
            tune.reset (0);
    }
    catch (...)
    {
        ;
    }

    QApplication::restoreOverrideCursor();

    if (!tune.get())
        return;

    setWindowFilePath (fileName);
    m_tune = tune;
    _loadTuneInfo ();
}

void SidEdit::_loadTuneInfo ()
{
    QString text;

    const SidTuneInfo &info = m_tune->getInfo ();
    uint_least16_t     song = info.currentSong;
    m_sbrSong->setValue  (song);
    m_spnSongs->setValue (info.songs);
    m_defaultSong = info.startSong;
    _defaultSong (song == m_defaultSong);

    QString str;
    str.setNum (song);
    m_txtSong->setText (str);

    // Load Address
    m_spnLoadAddress->setValue (info.loadAddr);
    text = QString::number(info.dataFileLen);
    if (info.dataFileLen > 0)
    {   // Add end address if usefull
        text += " (";
        text += tr (QT_TR_NOOP("End"));
        text += ": ";
        text += m_spnLoadAddress->prefix ();
        text += QString::number(info.loadAddr + info.dataFileLen - 1, 16).rightJustified(4, '0').toUpper ();
        text += ')';
    }
    m_txtLoadRange->setText    (text);
    m_spnInitAddress->setValue (info.initAddr);
    m_spnPlayAddress->setValue (info.playAddr);

    // Player Details
    m_spnPlayerStart->setValue (info.relocStartPage << 8);
    int start = info.relocStartPage << 8;
    int end   = 0;
    if ((start > 0) || (start < 0xff00))
        end = start + (info.relocPages << 8) - 1;
    m_spnPlayerStart->setValue (start);
    m_spnPlayerEnd->setValue   (end);
    _pages (start, end);

    // Credits
    QTextDecoder* decoder = QTextCodec::codecForName("Windows-1252")->makeDecoder ();
    if (info.numberOfInfoStrings > 0)
    {
        text = decoder->toUnicode (info.infoString[0], strlen(info.infoString[0]));
        m_txtCreditTitle->setText    (text);
    }
    if (info.numberOfInfoStrings > 1)
    {
        text = decoder->toUnicode (info.infoString[1], strlen(info.infoString[1]));
        m_txtCreditAuthor->setText   (text);
    }
    if (info.numberOfInfoStrings > 2)
    {
        text = decoder->toUnicode (info.infoString[2], strlen(info.infoString[2]));
        m_txtCreditReleased->setText (text);
    }

    m_cboPlayerType->setCurrentIndex (m_playerType.indexOf(info.compatibility));
    if (m_fileType == ftPSID)
    {
        int index = m_timingInfo.indexOf (info.songSpeed);
        m_cboTiming->setCurrentIndex     (index);
        _timingEnabled                   (song <= m_timingMaxSong);
    }

    m_cboSidType->setCurrentIndex    (m_sidTypeInfo.indexOf(info.sidModel1));
    m_cboSid2Type->setCurrentIndex   (m_sidTypeInfo.indexOf(info.sidModel2));
    m_spnSid2Address->setValue       (info.sidChipBase2);
    m_cboVicSpeed->setCurrentIndex   (m_vicSpeedInfo.indexOf(info.clockSpeed));

    bool enable = m_subtuneInfo ? true : song == 1;
    emit _subtune (enable);
}

void SidEdit::_new ()
{
    if (_maybeSave())
    {
        try
        {
            m_tune.reset (new SidTuneWrite(0));
            (void)m_tune->selectSong (1);
        }
        catch (...)
        {
            return;
        }

        setWindowFilePath ("");
        _loadTuneInfo ();
//    m_txtCreditTitle->setText    ("<?>");
//    m_txtCreditAuthor->setText   ("<?>");
//    m_txtCreditReleased->setText ("<?>");
    }
}

bool SidEdit::_maybeSave ()
{
    if (m_isModified)
    {
        QMessageBox::StandardButton ret;
        ret = QMessageBox::warning(this, tr("Application"),
                     tr("The document has been modified.\n"
                        "Do you want to save your changes?"),
                     QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if (ret == QMessageBox::Save)
            return _save ();
        else if (ret == QMessageBox::Cancel)
            return false;
    }
    return true;
}

void SidEdit::_open()
{
    if (_maybeSave())
    {
        QString fileName = QFileDialog::getOpenFileName (this);
        if (!fileName.isEmpty())
            _loadFile (fileName);
    }
}

void SidEdit::_pages (int start, int end)
{
    QString str;
    int offset = 0, length = 0;

    if (start < 0x100)
    {
        str += tr (QT_TR_NOOP("Auto"));
        end  = 0;
    }
    else if (start >= 0xff00)
    {
        str   += tr (QT_TR_NOOP("No Space"));
        offset = 0xff;
    }
    else
    {
        offset  = ((start + 0xff) >> 8);
        length  = ((end   - 0xff) >> 8);
        length -= offset;
        if ((end < start) || (length < 0))
        {
            m_txtPlayerPages->setText (tr(QT_TR_NOOP("Illegal")));
            return;
        }

        ++length;
        str += QString::number (length);
    }

    str += " (";
    str += tr (QT_TR_NOOP("Code"));
    str += ": ";
    str += m_spnPlayerStart->prefix ();
    str += QString::number (offset, 16).rightJustified(2, '0').toUpper ();
    str += QString::number (length, 16).rightJustified(2, '0').toUpper ();
    str += ')';
    m_txtPlayerPages->setText (str);
    m_pages = length;
}

void SidEdit::_playerEndChanged (int end)
{
    _pages (m_spnPlayerStart->value(), end);
}

void SidEdit::_playerStartChanged (int start)
{
    _pages (start, m_spnPlayerEnd->value());
}

void SidEdit::_playerTypeChanged (int index)
{
    if (index < 0)
        return;

    m_fileType = ftPSID;
    switch (m_playerType.value(index))
    {
    case SIDTUNE_COMPATIBILITY_R64:
    case SIDTUNE_COMPATIBILITY_BASIC:
        m_fileType = ftRSID;
    }

    switch (m_fileType)
    {
    case ftPSID:
        m_spnSongs->setMaximum   (256);
        m_timingMaxSong         = 32;
        m_subtuneInfo = false;
        emit _subtune (m_sbrSong->value() == 1);
        emit _timingEnabled (m_sbrSong->value() <= m_timingMaxSong);
        break;
    case ftRSID:
        {   // RSID must use VIC timing and re-program the hardware for CIA
            int index = m_timingInfo.indexOf (SIDTUNE_SPEED_VBI);
            m_cboTiming->setCurrentIndex (index);
        }
        m_spnSongs->setMaximum   (256);
        m_timingMaxSong         = 0;
        m_subtuneInfo = false;
        emit _subtune (m_sbrSong->value() == 1);
        emit _timingEnabled (false);
        m_spnInitAddress->setValue (0);
        m_spnPlayAddress->setValue (0);
        break;
    }
}

bool SidEdit::_save()
{
    bool ret = false;
    if (windowFilePath().isEmpty())
        ret = _saveAs ();
    else
        ret = _saveFile (windowFilePath());
    if (ret)
        m_isModified = false;
    return ret;
}

bool SidEdit::_saveAs ()
{
    QString fileName = QFileDialog::getSaveFileName (this);
    if (fileName.isEmpty())
        return false;
    return _saveFile (fileName);
}

bool SidEdit::_saveFile (QString fileName)
{
    if (fileName.isEmpty())
        fileName = "untitled";

    for (;;)
    {
        bool ret;
        if (!_saveTuneInfo(m_sbrSong->value()))
            break;

        QApplication::setOverrideCursor(Qt::WaitCursor);
        ret = m_tune->savePSIDfile (fileName.toUtf8(), true);
        QApplication::restoreOverrideCursor();
        if (!ret)
            break;

        setWindowFilePath (fileName);
        return true;
    }
    QMessageBox::critical (this, tr("Application"), m_tune->getInfo().statusString);
    return false;
}

bool SidEdit::_saveTuneInfo (int song)
{
    QString text;

    SidTuneInfo info = m_tune->getInfo ();
    info.currentSong = song;
    info.startSong   = m_defaultSong;
    info.songs       = m_spnSongs->value ();

    info.compatibility = m_playerType.value  (m_cboPlayerType->currentIndex());
    info.sidModel1     = m_sidTypeInfo.value (m_cboSidType->currentIndex());
    info.sidChipBase1  = 0xd400;
    info.sidModel2     = m_sidTypeInfo.value (m_cboSid2Type->currentIndex());
    info.sidChipBase2  = m_spnSid2Address->value ();

    info.songSpeed  = m_timingInfo.value   (m_cboTiming->currentIndex());
    info.clockSpeed = m_vicSpeedInfo.value (m_cboVicSpeed->currentIndex());

    // Load Address
    info.fixLoad  = false;
    info.loadAddr = m_spnLoadAddress->value ();
    info.initAddr = m_spnInitAddress->value ();
    info.playAddr = m_spnPlayAddress->value ();

    // Player Details
    info.relocStartPage = (m_spnPlayerStart->value() + 0xff) >> 8;
    info.relocPages     = m_pages;

    // Credits
    QTextEncoder* encoder    = QTextCodec::codecForName("Windows-1252")->makeEncoder ();
    info.numberOfInfoStrings = 3;
    char infoString[3][SIDTUNE_MAX_CREDIT_STRLEN];

    strncpy (infoString[0], encoder->fromUnicode(m_txtCreditTitle->text()),    SIDTUNE_MAX_CREDIT_STRLEN);
    strncpy (infoString[1], encoder->fromUnicode(m_txtCreditAuthor->text()),   SIDTUNE_MAX_CREDIT_STRLEN);
    strncpy (infoString[2], encoder->fromUnicode(m_txtCreditReleased->text()), SIDTUNE_MAX_CREDIT_STRLEN);

    // Clear other fields
    infoString[0][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';
    infoString[1][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';
    infoString[2][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';

    // Assign Strings
    info.infoString[0] = infoString[0];
    info.infoString[1] = infoString[1];
    info.infoString[2] = infoString[2];

    return m_tune->setInfo (info);
}

void SidEdit::_songChanged (int song)
{
    uint_least16_t currentSong = m_tune->getInfo().currentSong;
    if (!_saveTuneInfo(currentSong))
    {
        m_sbrSong->setValue (currentSong);
        return;
    }
    currentSong = m_tune->selectSong (song);
    if (song != currentSong)
    {
        m_sbrSong->setValue (currentSong);
        return;
    }

    _loadTuneInfo ();
}

void SidEdit::_songsChanged (int songs)
{
    if (m_defaultSong > songs)
        m_defaultSong = 1;
    bool increase = (songs > m_sbrSong->maximum());
    m_sbrSong->setMaximum (songs);
    if (increase)
        m_sbrSong->setValue (songs);
}

void SidEdit::_updateComboBox (QComboBox *cbo, EnumInfo &info)
{
    cbo->clear ();
    for (int i = 0; i < info.size(); ++i)
        cbo->addItem (tr(info.at(i)));
}
