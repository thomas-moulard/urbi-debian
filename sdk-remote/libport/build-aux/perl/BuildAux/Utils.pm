package BuildAux::Utils;

=head1 NAME

BuildAux::Utils - same utility functions

=head1 SYNOPSIS

  use BuildAux::Utils;

=head1 DESCRIPTION

This perl module provides various general purpose functions.

=cut

use strict;
use BuildAux::Verbose;
use Exporter;

our @ISA = qw (Exporter);
our @EXPORT = qw (&file_comment_type &file_type
                  &license
                  &make_comment &unique &xsystem);


=item C<file_comment_type ($file)>

The type of comment to use for C<$file>.  Return C<c> (including for
C<c++> and C<urbiscript>), C<sh> (including for C<makefile>) or
C<tex>.

=cut

sub file_comment_type ($)
{
  my ($file) = @_;
  # The comment type to use.
  my $res = file_type($file);
  if ($res)
  {
    $res = 'c'
      if $res eq 'c++' || $res eq 'urbiscript';
    $res = 'sh'
      if $res eq 'makefile';
  }
  $res;
}


=item C<file_type ($file)>

The type of C<$file>, based on its name.  Return values: C<c>, C<c++>,
C<urbiscript>, C<tex>.

=cut

sub file_type ($)
{
  my ($file) = @_;
  $file =~ s/\.in\z//;
  (my $ext = $file) =~ s{.*\.}{};
  my $res;
  if ($ext =~ m{\A(?:h|c)\z})
  {
    $res = 'c';
  }
  elsif ($ext =~ m{\A(?:cc|hh|hxx)\z})
  {
    $res = 'c++';
  }
  elsif ($ext =~ m{\A(?:am|mk)\z})
  {
    $res = 'makefile';
  }
  elsif ($ext =~ m{\A(?:ac|m4)\z})
  {
    $res = 'm4';
  }
  elsif ($ext eq 'tex')
  {
    $res = 'tex';
  }
  elsif ($ext eq 'u')
  {
    $res = 'urbiscript';
  }
  $res;
}


=item C<make_box($prologue, $prefix, $epilogue, $body)>

Return C<$body> with each line prefixed by C<$prefix>, and with
C<$prologue> before and C<$epilogue> after, both on single lines.

=cut

sub make_box($$$$)
{
  my ($prologue, $prefix, $epilogue, $body) = @_;
  my $res = $body;
  $res =~ s/^/$prefix/gm;
  $res = $prologue . "\n" . $res
    if $prologue;
  $res .= $epilogue . "\n"
    if $epilogue;

  # Strip trailing white spaces.
  $res =~ s/[\t ]+$//mg;

  $res;
}


=item C<make_comment($language, $body)>

Return C<$body> turned into a comment for C<$language>.  C<$language>
can be C<c> (C</*...*/), C<c++> (C<// ...>), or C<tex> (C<%% ...>).

=cut

sub make_comment($$)
{
  my ($language, $body) = @_;
  my %markup =
    (
     c     => ['/*', ' * ', ' */'],
     'c++' => ['',   '// ', ''],
     m4    => ['',   '## ', ''],
     sh    => ['',   '## ', ''],
     tex   => ['',   '%% ', ''],
    );
  my ($pre, $in, $post) = @{$markup{$language}};
  make_box($pre, $in, $post, $body);
}

=item C<license($language, [$years = current-year])>

The Gostai license file header for C<$years>, as a comment for
C<$language>.  See C<make_comment> for the available languages.

=cut

sub license
{
  my ($language, $years) = @_;
  $years = 1900 + @{[localtime]}[5]
    unless defined $years;

  my $res = make_comment($language, <<EOF);
Copyright (C) $years, Gostai S.A.S.

This software is provided "as is" without warranty of any kind,
either expressed or implied, including but not limited to the
implied warranties of fitness for a particular purpose.

See the LICENSE file for more information.
EOF
}



=item C<unique(@list)>

Return C<@list> removing duplicates.

=cut

sub unique (@)
{
  my (@list) = @_;
  my %seen;
  my @res;
  for my $item (@list)
  {
    if (! exists $seen{$item})
    {
      $seen{$item} = 1;
      push @res, $item;
    }
  }
  @res;
}

=item C<xsystem(@argv)>

Run C<system(@argv)>.  Fail on failure.

=cut

sub xsystem (@)
{
  my (@args) = @_;

  verbose 2, "running: @args";
  system(@args) == 0
    or die "$me: system @args failed: $?\n";
  if ($? == -1)
  {
    die "$me: failed to execute: $!\n";
  }
  elsif ($? & 127)
  {
    die sprintf ("$me: child died with signal %d, %s coredump\n",
                 ($? & 127),  ($? & 128) ? 'with' : 'without');
  }
  elsif ($? >> 8)
  {
    die sprintf "$me: child exited with value %d\n", $? >> 8;
  }
}

1; # for require

### Setup "Gostai" style for perl-mode and cperl-mode.
## Local Variables:
## perl-indent-level: 2
## perl-continued-statement-offset: 2
## perl-continued-brace-offset: -2
## perl-brace-offset: 0
## perl-brace-imaginary-offset: 0
## perl-label-offset: -2
## cperl-indent-level: 2
## cperl-brace-offset: 0
## cperl-continued-brace-offset: -2
## cperl-label-offset: -2
## cperl-extra-newline-before-brace: t
## cperl-merge-trailing-else: nil
## cperl-continued-statement-offset: 2
## End:
