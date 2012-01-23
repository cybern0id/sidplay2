#ifndef SIDEDIT_H
#define SIDEDIT_H

#include <algorithm>

#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QScrollBar>
#include <QSpinBox>

#include "SidTuneWrite.h"
#include "enuminfo.h"
#include "hexspinbox.h"

class SidEdit : public QMainWindow
{
    Q_OBJECT

public:
    typedef enum {ftPSID, ftRSID} filetype_t;
    enum { MUSPLAYER = -1 };
    struct PlayerType:   TEnumInfo<int> { PlayerType (); };
    struct SIDTypeInfo:  TEnumInfo<int> { SIDTypeInfo (); };
    struct TimingInfo:   TEnumInfo<int> { TimingInfo (); };
    struct VICSpeedInfo: TEnumInfo<int> { VICSpeedInfo (); };

    SidEdit(QWidget *parent = 0, Qt::WFlags flags = 0);
    ~SidEdit();

protected:
    virtual void closeEvent (QCloseEvent *closeEvent);

private:
    template <class T> T* _find (const char *objectName);
    void _fileTypeChanged (filetype_t fileType);
    void _loadFile        (const QString &fileName);
    void _loadTuneInfo    ();
    bool _maybeSave       ();
    void _pages           (int start, int end);
    bool _saveFile        (QString fileName);
    void _updateComboBox  (QComboBox *cbo, EnumInfo &info);

private slots:
    void _defaultClicked     ();
    void _new                ();
    void _open               ();
    void _playerEndChanged   (int end);
    void _playerStartChanged (int end);
    void _playerTypeChanged  (int index);
    bool _save               ();
    bool _saveAs             ();
    bool _saveTuneInfo       (int song);
    void _songChanged        (int song);
    void _songsChanged       (int songs);

signals:
    void _defaultSong   (int  index);
    void _subtune       (bool visible);
    void _timingEnabled (bool enabled);

private:
    int                    m_defaultSong;
    filetype_t             m_fileType;
    bool                   m_isModified;
    uint_least8_t          m_pages;
    PlayerType             m_playerType;
    SIDTypeInfo            m_sidTypeInfo;
    bool                   m_subtuneInfo;
    TimingInfo             m_timingInfo;
    int                    m_timingMaxSong;
    std::auto_ptr<SidTuneWrite> m_tune;
    VICSpeedInfo           m_vicSpeedInfo;

    QComboBox  *m_cboPlayerType;
    QComboBox  *m_cboSidType;
    QComboBox  *m_cboSid2Type;
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
    HexSpinBox *m_spnSid2Address;
    QSpinBox   *m_spnSongs;
    QLabel     *m_txtPlayerPages;
    QLabel     *m_txtSong;
};

#endif // SIDEDIT_H
