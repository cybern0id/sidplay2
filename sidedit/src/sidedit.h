#ifndef SIDEDIT_H
#define SIDEDIT_H

#include <algorithm>

#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QScrollBar>
#include <QSpinBox>

#include <sidplay/SidTune.h>
#include "enuminfo.h"
#include "hexspinbox.h"

class SidEdit : public QMainWindow
{
    Q_OBJECT

public:
    typedef enum {ftPSID, ftRSID} filetype_t;
    enum { MUSPLAYER = -1 };
    struct FileTypeInfo: TEnumInfo<SidEdit::filetype_t> { FileTypeInfo (); };
    struct PlayerType:   TEnumInfo<int> { PlayerType (bool rsid); };
    struct SIDTypeInfo:  TEnumInfo<int> { SIDTypeInfo (); };
    struct TimingInfo:   TEnumInfo<int> { TimingInfo (); };
    struct VICSpeedInfo: TEnumInfo<int> { VICSpeedInfo (); };

    SidEdit(QWidget *parent = 0, Qt::WFlags flags = 0);
    ~SidEdit();

protected:
    virtual void closeEvent (QCloseEvent *closeEvent);

private:
    template <class T> T* _find (const char *objectName);
    void _loadFile       (const QString &fileName);
    void _loadTuneInfo   ();
    bool _maybeSave      ();
    void _pages          (int start, int end);
    bool _saveFile       (QString fileName);
    void _setupPSID      (int version);
    void _setupRSID      (int version);
    void _updateComboBox (QComboBox *cbo, EnumInfo &info);

private slots:
    void _defaultClicked     ();
    void _fileTypeChanged    (int index);
    void _new                ();
    void _open               ();
    void _playerEndChanged   (int end);
    void _playerStartChanged (int end);
    bool _save               ();
    bool _saveAs             ();
    void _songChanged        (int song);
    void _songsChanged       (int songs);
    void _versionChanged     (int version);

signals:
    void _defaultSong   (int  index);
    void _player        (bool visible);
    void _player        (int  index);
    void _hwInfo        (bool visible);
    void _subtune       (bool visible);
    void _subtuneHwInfo (bool visible);
    void _timingEnabled (bool enabled);

private:
    int                    m_defaultSong;
    filetype_t             m_fileType;
    FileTypeInfo           m_fileTypeInfo;
    bool                   m_hwInfo;
    bool                   m_isModified;
    PlayerType             m_playerTypePSID;
    PlayerType             m_playerTypeRSID;
    SIDTypeInfo            m_sidTypeInfo;
    bool                   m_subtuneInfo;
    TimingInfo             m_timingInfo;
    int                    m_timingMaxSong;
    std::auto_ptr<SidTune> m_tune;
    VICSpeedInfo           m_vicSpeedInfo;
    void (SidEdit::*       m_setup) (int);

    QComboBox  *m_cboFileType;
    QComboBox  *m_cboPlayerType;
    QComboBox  *m_cboSidType;
    QComboBox  *m_cboTiming;
    QComboBox  *m_cboVicSpeed;
    QLineEdit  *m_txtCreditTitle;
    QLineEdit  *m_txtCreditAuthor;
    QLineEdit  *m_txtCreditReleased;
    QLabel     *m_txtLoadRange;
    QScrollBar *m_sbrSong;
    HexSpinBox *m_spnInitAddress;
    HexSpinBox *m_spnLoadAddress;
    HexSpinBox *m_spnPlayAddress;
    HexSpinBox *m_spnPlayerStart;
    HexSpinBox *m_spnPlayerEnd;
    QSpinBox   *m_spnSongs;
    QSpinBox   *m_spnVersion;
    QLabel     *m_txtPlayerPages;
    QLabel     *m_txtSong;
};

#endif // SIDEDIT_H
