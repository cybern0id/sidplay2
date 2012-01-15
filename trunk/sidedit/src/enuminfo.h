#ifndef ENUMINFO_H
#define ENUMINFO_H

#include <QPair>
#include <QString>
#include <QVector>

class EnumInfo
{
public:
    virtual ~EnumInfo () { ; }

    virtual const char *at   (int index) const = 0;
    virtual int         size () const = 0;

protected:
    EnumInfo () { ; }
};

template <typename T>
class TEnumInfo: public EnumInfo
{
public:
    virtual int size () const { return m_info.size(); }

    virtual const char *at (int index) const
    {
        return m_info.at(index).second;
    }

    T   value   (int index) const { return m_info.at(index).first; }
    int indexOf (T value) const
    {
        for (int i = 0; i < m_info.size(); ++i)
        {
            if (m_info[i].first == value)
                return i;
        }
        return -1;
    }

protected:
    void _add (T val, const char *description)
    {
        m_info.push_back (qMakePair(val, description));
    }

private:
    typedef QPair<T, const char *> EnumInfo;

    QVector<EnumInfo> m_info;
};

#endif // ENUMINFO_H
