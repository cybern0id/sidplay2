%define name    libsidutils
%define version 1.0.0
%define release 1
%define major   1

Summary:        General utility library for use in sidplayers.
Name:           %{name}
Version:        %{version}
Release:        %{release}
Source:         %{name}-%{version}.tar.bz2
Copyright:      GPL
Group:          System/Libraries
URL:            http://sidplay2.sourceforge.net/
BuildRoot:      %{_tmppath}/%{name}%{major}-buildroot
Prefix:         %{_prefix}
Requires:       libsidplay >= 2.0.7

%description
This library provides general utilities that are not considered core
to the C64 emulation.  Utilities include decoding and obtaining tune
lengths from the songlength database, INI file format parser and SID
filter files (types 1 and 2).

%package devel
Summary:        Development headers and libraries for %{name}%{major}
Group:          Development/C++

%description devel
This package includes the header and library files necessary
for developing applications to use %{name}%{major}.


%prep
rm -rf $RPM_BUILD_ROOT 
%setup -q

%build
CXXFLAGS="$RPM_OPT_FLAGS" ./configure --prefix=%{_prefix}
make

%install
make DESTDIR=$RPM_BUILD_ROOT install

%clean
rm -rf $RPM_BUILD_ROOT

%postun
/sbin/ldconfig

%post
/sbin/ldconfig

%files
%defattr(-,root,root)
%doc AUTHORS COPYING ChangeLog README TODO
%{_libdir}/*.so.*

%files devel
%defattr(-,root,root)
%{_includedir}/sidplay/utils/*
%{_libdir}/*.la
%{_libdir}/*.a
%{_libdir}/*.so

%changelog
* Wed Apr 10 2001 Simon White <s_a_white@email.com> 1.0.0-1
- Use non Mandrake specific release number.  Bug fixes in INI
  file parser.

* Wed Apr 4 2001 Simon White <s_a_white@email.com> 1.0.0-2mdk
- Updated --prefix and make install so la file does not end up with
  a bad install path.

* Sun Apr 1 2001 Simon White <s_a_white@email.com> 1.0.0-1mdk
- First spec file.

# end of file
