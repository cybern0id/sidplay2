#ifndef SIDEDIT_H
#define SIDEDIT_H

#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QScrollBar>
#include <QSpinBox>

#include <sidplay/SidTune.h>
#include "enuminfo.h"

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

private:
    template <class T> T* _find (const char *objectName);
    void _setupPSID      (int version);
    void _setupRSID      (int version);
    void _updateComboBox (QComboBox *cbo, EnumInfo &info);

private:
    int              m_defaultSong;
    filetype_t       m_fileType;
    FileTypeInfo     m_fileTypeInfo;
    bool             m_hwInfo;
    PlayerType       m_playerTypePSID;
    PlayerType       m_playerTypeRSID;
    SIDTypeInfo      m_sidTypeInfo;
    bool             m_subtuneInfo;
    TimingInfo       m_timingInfo;
    int              m_timingMaxSong;
    VICSpeedInfo     m_vicSpeedInfo;
    void (SidEdit::* m_setup) (int);

    QComboBox  *m_cboFileType;
    QComboBox  *m_cboPlayerType;
    QComboBox  *m_cboSidType;
    QComboBox  *m_cboTiming;
    QComboBox  *m_cboVicSpeed;
    QLineEdit  *m_txtCreditTitle;
    QLineEdit  *m_txtCreditAuthor;
    QLineEdit  *m_txtCreditReleased;
    QScrollBar *m_sbrSong;
    QSpinBox   *m_spnSongs;
    QSpinBox   *m_spnVersion;
    QLabel     *m_txtPlayerPages;
    QLabel     *m_txtSong;

private slots:
    void _defaultClicked  ();
    void _fileTypeChanged (int index);
    void _new             ();
    void _songChanged     (int song);
    void _songsChanged    (int songs);
    void _versionChanged  (int version);

signals:
    void _defaultSong   (int  index);
    void _player        (bool visible);
    void _player        (int  index);
    void _hwInfo        (bool visible);
    void _subtune       (bool visible);
    void _subtuneHwInfo (bool visible);
    void _timingEnabled (bool enabled);
};

#endif // SIDEDIT_H
