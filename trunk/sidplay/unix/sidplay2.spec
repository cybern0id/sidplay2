%define name    sidplay
%define version 2.0.7
%define release 2
%define major   2

Summary:        A Commodore 64 music player and SID chip emulator.
Name:           %{name}
Version:        %{version}
Release:        %{release}
Source:         %{name}-%{version}.tar.bz2
Copyright:      GPL
Group:          Applications/Multimedia
URL:            http://sidplay2.sourceforge.net/
BuildRoot:      %{_tmppath}/%{name}%{major}-buildroot
Prefix:         %{_prefix}
Requires:       libsidplay >= 2.0.7 libsidutils >= 1.0.0

%description
Sidplay2 is the second in the Sidplay series and provides a console front end for the
libsidplay2 library.  This library is cycle accurate for improved sound reproduction
and is capable of playing all C64 mono and stereo file formats.  Also supported is a
full C64 emulation environment, which allows tunes to be taken directly from the C64
without the need for special modifications.

%prep
rm -rf $RPM_BUILD_ROOT 
%setup -q

%build
CXXFLAGS="$RPM_OPT_FLAGS" ./configure --prefix=%{_prefix} --mandir=%{_mandir}
make

%install
make DESTDIR=$RPM_BUILD_ROOT install

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(-,root,root)
%doc AUTHORS COPYING ChangeLog README TODO
%{_mandir}/man*/*
%{_bindir}/*

%changelog
* Wed Apr 10 2001 Simon White <s_a_white@email.com> 2.0.7-2
- Use non Mandrake specific release number.

* Sun Apr 1 2001 Simon White <s_a_white@email.com> 2.0.7-1mdk
- First spec file.

# end of file
