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


SidEdit::PlayerType::PlayerType (bool rsid)
{
    if (rsid)
        _add (SIDTUNE_COMPATIBILITY_R64,   QT_TR_NOOP("C64 Only"));
    else
        _add (SIDTUNE_COMPATIBILITY_C64,   QT_TR_NOOP("C64 Compatible"));
    if (rsid)
        _add (SIDTUNE_COMPATIBILITY_BASIC, QT_TR_NOOP("C64 Basic"));
    else
        _add (SIDTUNE_COMPATIBILITY_PSID,  QT_TR_NOOP("PlaySID Only"));
    _add (MUSPLAYER, QT_TR_NOOP("Compute's Sidplayer"));
}

SidEdit::FileTypeInfo::FileTypeInfo ()
{
    _add (SidEdit::ftPSID, "PSID");
    _add (SidEdit::ftRSID, "RSID");
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
,m_playerTypePSID(false)
,m_playerTypeRSID(true)
,m_setup(0)
,m_tune(new SidTune(0))
{
    {   // Do this to ensure we are compatible with dynamic loading
        Ui::sideditClass ui;
        ui.setupUi(this);
    }

    m_cboFileType       = _find<QComboBox>("cboFileType");
    m_cboPlayerType     = _find<QComboBox>("cboPlayerType");
    m_cboSidType        = _find<QComboBox>("cboSidType");
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
    m_spnVersion        = _find<QSpinBox>("spnVersion");
    m_spnSongs          = _find<QSpinBox>("spnSongs");
    m_spnVersion        = _find<QSpinBox>("spnVersion");
    m_txtPlayerPages    = _find<QLabel>("txtPlayerPages");
    m_txtSong           = _find<QLabel>("txtSong");

    {
        QLabel *txtPlayerType = _find<QLabel>("txtPlayerType");
        int index = m_playerTypePSID.indexOf(SIDTUNE_COMPATIBILITY_PSID);
        txtPlayerType->setText (m_playerTypePSID.at(index));
    }

    connect (m_cboFileType, SIGNAL(currentIndexChanged(int)), SLOT(_fileTypeChanged(int)));
    connect (m_spnVersion,  SIGNAL(valueChanged(int)), SLOT(_versionChanged(int)));

    connect (this, SIGNAL(_player(int)),  _find<QObject>("stkPlayerType"),  SLOT(setCurrentIndex(int)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("stkPlayerType"),  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("lblPlayerType"),  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("lblPlayerTitle"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("lblPlayerStart"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("lblPlayerEnd"),   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), _find<QObject>("lblPlayerRange"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), m_spnPlayerStart, SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), m_spnPlayerEnd,   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_player(bool)), m_txtPlayerPages, SLOT(setEnabled(bool)));
    connect (m_spnPlayerStart, SIGNAL(valueChanged(int)), SLOT (_playerStartChanged(int)));
    connect (m_spnPlayerEnd,   SIGNAL(valueChanged(int)), SLOT (_playerEndChanged(int)));

    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditTitle"),    SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditAuthor"),   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), _find<QObject>("lblCreditReleased"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditTitle,    SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditAuthor,   SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtune(bool)), m_txtCreditReleased, SLOT(setEnabled(bool)));

    connect (m_spnSongs, SIGNAL(valueChanged(int)), this, SLOT(_songsChanged(int)));
    connect (m_sbrSong,  SIGNAL(valueChanged(int)), this, SLOT(_songChanged(int)));

    connect (this, SIGNAL(_subtuneHwInfo(bool)), _find<QObject>("lblSidType"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtuneHwInfo(bool)), m_cboSidType,  SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtuneHwInfo(bool)), _find<QObject>("lblVicSpeed"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_subtuneHwInfo(bool)), m_cboVicSpeed, SLOT(setEnabled(bool)));

    connect (_find<QObject>("btnDefaultSong"), SIGNAL(clicked()), this, SLOT(_defaultClicked()));
    connect (this, SIGNAL(_defaultSong(int)), _find<QObject>("stkDefaultSong"), SLOT(setCurrentIndex(int)));

    connect (this, SIGNAL(_timingEnabled(bool)), _find<QObject>("lblTiming"), SLOT(setEnabled(bool)));
    connect (this, SIGNAL(_timingEnabled(bool)), m_cboTiming, SLOT(setEnabled(bool)));

    _new ();

//QTextEncoder* encoder = QTextCodec::codecForName("Windows-1250")->makeEncoder();
//QByteArray outputData = encoder->fromUnicode(result);
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

void SidEdit::_fileTypeChanged (int index)
{
    if (index >= 0)
    {
        m_fileType = m_fileTypeInfo.value (index);

        switch (m_fileType)
        {
        case ftPSID:
            m_spnSongs->setMaximum   (256);
            m_timingMaxSong         = 32;
            _updateComboBox (m_cboPlayerType, m_playerTypePSID);
            m_setup = &SidEdit::_setupPSID;
            m_spnVersion->setMinimum (1);
            m_spnVersion->setMaximum (3);
            m_spnVersion->setValue   (2);
            break;
        case ftRSID:
            {   // RSID must use VIC timing and re-program the hardware for CIA
                int index = m_timingInfo.indexOf (SIDTUNE_SPEED_VBI);
                m_cboTiming->setCurrentIndex (index);
            }
            m_spnSongs->setMaximum   (256);
            m_timingMaxSong         = 0;
            _updateComboBox (m_cboPlayerType, m_playerTypeRSID);
            m_setup = &SidEdit::_setupRSID;
            m_spnVersion->setMinimum (2);
            m_spnVersion->setMaximum (3);
            m_spnVersion->setValue   (2);
            break;
        }

        _versionChanged (m_spnVersion->value());
        _timingEnabled  (m_sbrSong->value() <= m_timingMaxSong);
    }
}

void SidEdit::_loadFile (const QString &fileName)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    std::auto_ptr<SidTune> tune;

    try
    {
        QFileInfo info(fileName);
        tune.reset (new SidTune(fileName.toUtf8()));

/*
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(fileName)
                             .arg(file.errorString()));
        return;
    }
*/
        if (!tune->getStatus())
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
    m_sbrSong->setValue   (info.startSong);
    m_spnSongs->setValue  (info.songs);

    switch (info.compatibility)
    {
    case SIDTUNE_COMPATIBILITY_R64:
    case SIDTUNE_COMPATIBILITY_BASIC:
        m_cboFileType->setCurrentIndex (m_fileTypeInfo.indexOf(ftRSID));
        break;
    default:
        m_cboFileType->setCurrentIndex (m_fileTypeInfo.indexOf(ftPSID));
        break;
    }

    // @FIXME@: Need to remove version setting and make it automatic
    m_spnVersion->setValue (3);

    _updateComboBox (m_cboFileType, m_fileTypeInfo);
    _updateComboBox (m_cboSidType,  m_sidTypeInfo);
    _updateComboBox (m_cboTiming,   m_timingInfo);
    _updateComboBox (m_cboVicSpeed, m_vicSpeedInfo);
    _defaultClicked ();

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

    if (m_fileType == ftRSID)
        m_cboPlayerType->setCurrentIndex (m_playerTypeRSID.indexOf(info.compatibility));
    else
    {
        m_cboPlayerType->setCurrentIndex (m_playerTypePSID.indexOf(info.compatibility));
        int index = m_timingInfo.indexOf (info.songSpeed);
        m_cboTiming->setCurrentIndex     (index);
    }

    m_cboSidType->setCurrentIndex    (m_sidTypeInfo.indexOf(info.sidModel1));
    m_cboVicSpeed->setCurrentIndex   (m_vicSpeedInfo.indexOf(info.clockSpeed));
}

void SidEdit::_new ()
{
    if (_maybeSave())
    {
        try
        {
            m_tune.reset (new SidTune(0));
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

        str += QString::number (length + 1);
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
    QApplication::setOverrideCursor(Qt::WaitCursor);

    QApplication::restoreOverrideCursor();

/*
    QMessageBox::warning(this, tr("Application"),
                         tr("Cannot write file %1:\n%2.")
                         .arg(fileName)
                         .arg(file.errorString()));
*/

    setWindowFilePath (fileName);
    return true;
}

bool SidEdit::_saveTuneInfo ()
{
    QString text;

    SidTuneInfo info = m_tune->getInfo ();
    info.currentSong = m_sbrSong->value ();
    info.startSong   = m_defaultSong;
    info.songs       = m_sbrSong->value ();

    if (m_fileType == ftRSID)
        info.compatibility = m_playerTypeRSID.value (m_cboPlayerType->currentIndex());
    else
        info.compatibility = m_playerTypePSID.value (m_cboPlayerType->currentIndex());

    info.sidModel1    = m_sidTypeInfo.value (m_cboSidType->currentIndex());
    info.sidChipBase1 = 0xd400;
//    info.sidModel2    = m_sidTypeInfo.value (m_cboSidType->currentIndex());
//    info.sidChipBase2 = 0xd400;

    info.songSpeed  = m_timingInfo.value   (m_cboTiming->currentIndex());
    info.clockSpeed = m_vicSpeedInfo.value (m_cboVicSpeed->currentIndex());

    // Load Address
    info.fixLoad  = false;
    info.loadAddr = m_spnLoadAddress->value ();
    info.initAddr = m_spnInitAddress->value ();
    info.playAddr = m_spnPlayAddress->value ();

    // Player Details
    info.relocStartPage = m_spnPlayerStart->value ();
    info.relocPages     = m_pages;

    // Credits
    QTextEncoder* encoder    = QTextCodec::codecForName("Windows-1252")->makeEncoder ();
    info.numberOfInfoStrings = 3;
    strncpy (info.infoString[0], encoder->fromUnicode(m_txtCreditTitle->text()),    SIDTUNE_MAX_CREDIT_STRLEN);
    strncpy (info.infoString[1], encoder->fromUnicode(m_txtCreditAuthor->text()),   SIDTUNE_MAX_CREDIT_STRLEN);
    strncpy (info.infoString[2], encoder->fromUnicode(m_txtCreditReleased->text()), SIDTUNE_MAX_CREDIT_STRLEN);

    // Clear other fields
    info.infoString[0][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';
    info.infoString[1][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';
    info.infoString[2][SIDTUNE_MAX_CREDIT_STRLEN-1] = '\0';

    for ( uint_least16_t sNum = 3; sNum < SIDTUNE_MAX_CREDIT_STRINGS; sNum++ )
    {
        for ( uint_least16_t sPos = 0; sPos < SIDTUNE_MAX_CREDIT_STRLEN; sPos++ )
            info.infoString[sNum][sPos] = 0;
    }

    return true;
}

void SidEdit::_setupPSID (int version)
{
    int version2  = version > 1;
    m_hwInfo      = version2;
    m_subtuneInfo = false;
    emit _hwInfo (m_hwInfo);
    emit _player (version2);
    emit _player ((int)version2);
    emit _subtuneHwInfo (m_hwInfo && (m_sbrSong->value() == 1));

    if (!version2)
    {
        m_cboSidType->setCurrentIndex  (m_sidTypeInfo.indexOf(SIDTUNE_SIDMODEL_UNKNOWN));
        m_cboVicSpeed->setCurrentIndex (m_vicSpeedInfo.indexOf(SIDTUNE_CLOCK_UNKNOWN));
    }
}

void SidEdit::_setupRSID (int version)
{
    m_hwInfo      = true;
    m_subtuneInfo = false;
    emit _hwInfo (m_hwInfo);
    emit _player (true);
    emit _player ((int)true);
    emit _subtuneHwInfo (m_sbrSong->value() == 1);
    m_spnInitAddress->setValue (0);
    m_spnPlayAddress->setValue (0);
}

void SidEdit::_songChanged (int song)
{
    bool enable = m_subtuneInfo ? true : song == 1;
    emit _subtune       (enable);
    emit _subtuneHwInfo (m_hwInfo && enable);


    QString str;
    str.setNum (song);
    m_txtSong->setText (str);
    _defaultSong   (song == m_defaultSong);
    _timingEnabled (song <= m_timingMaxSong);
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

void SidEdit::_versionChanged (int version)
{
    if (m_setup)
        (this->*m_setup) (version);
}

void SidEdit::_updateComboBox (QComboBox *cbo, EnumInfo &info)
{
    cbo->clear ();
    for (int i = 0; i < info.size(); ++i)
        cbo->addItem (tr(info.at(i)));
}
