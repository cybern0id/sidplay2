#include <QCloseEvent>
#include <QFileDialog>
#include <QMessageBox>

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
        _add (SIDTUNE_COMPATIBILITY_C64,   QT_TR_NOOP("C64 Only"));
    else
        _add (SIDTUNE_COMPATIBILITY_R64,   QT_TR_NOOP("C64 Compatible"));
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
,m_playerTypePSID(false)
,m_playerTypeRSID(true)
,m_setup(0)
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

//QTextDecoder* decoder = QTextCodec::codecForName("Windows-1252")->makeDecoder();
//    QString result = decoder->toUnicode(data,data.length());
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

void SidEdit::_loadFile (const QString &fileName)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    QApplication::restoreOverrideCursor();

/*
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(fileName)
                             .arg(file.errorString()));
        return;
    }
*/
    setWindowFilePath (fileName);
}

void SidEdit::_new ()
{
    if (_maybeSave())
    {
        setWindowFilePath ("");
        m_sbrSong->setValue  (1);
        m_spnSongs->setValue (1);
        _updateComboBox (m_cboFileType, m_fileTypeInfo);
        _updateComboBox (m_cboSidType,  m_sidTypeInfo);
        _updateComboBox (m_cboTiming,   m_timingInfo);
        _updateComboBox (m_cboVicSpeed, m_vicSpeedInfo);
        _defaultClicked ();

        // Load Address
        m_spnLoadAddress->setValue (0);
        m_txtLoadRange->setText    ("1 (End: 0x0000)");
        m_spnInitAddress->setValue (0);
        m_spnPlayAddress->setValue (0);

        // Player Details
        m_spnPlayerStart->setValue (0);
        m_spnPlayerEnd->setValue   (0);
        _pages (0, 0);

        // Credits
        m_txtCreditTitle->setText    ("<?>");
        m_txtCreditAuthor->setText   ("<?>");
        m_txtCreditReleased->setText ("<?>");

        m_isModified = false;
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

    if ((start | end) == 0)
        str += QT_TR_NOOP("Auto");
    else
    {
        offset  = ((start + 0xff) >> 8);
        length  = ((end   - 0xff) >> 8);
        length -= offset;
        if ((end < start) || (length < 0))
        {
            m_txtPlayerPages->setText (QT_TR_NOOP("Illegal"));
            return;
        }

        str += QString::number (length + 1);
    }

    str += " (";
    str += QT_TR_NOOP("Code");
    str += ": ";
    str += m_spnPlayerStart->prefix ();
    str += QString::number (offset, 16).rightJustified(2, '0').toUpper ();
    str += QString::number (length, 16).rightJustified(2, '0').toUpper ();
    str += ')';
    m_txtPlayerPages->setText (str);
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
    if (windowFilePath().isEmpty())
        return _saveAs ();
    return _saveFile (windowFilePath());
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
        cbo->addItem (info.at(i));
}
