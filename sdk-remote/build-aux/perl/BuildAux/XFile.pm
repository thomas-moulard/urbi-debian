package BuildAux::XFile;

=head1 NAME

BuildAux::XFile - Wrap IO::File with verbosity and error handling

=head1 SYNOPSIS

  use BuildAux::XFile;

=head1 DESCRIPTION

Inherit from IO::File and provide tracing and error handling features.

=cut

use strict;
use IO::File;
use BuildAux::Verbose;

use Exporter;

our @ISA = qw(IO::File Exporter DynaLoader);

our @EXPORT = @IO::File::EXPORT;

=item C<new>

=cut

sub new
{
  my $self = shift;
  my $class = ref $self || $self || "BuildAux::XFile";
  my $res = $class->SUPER::new();
  $res->open(@_)
    if @_;
  $res;
}

=item C<open($file)>

Open C<$file>, for reading or writing.  It may be a command, if ended
by a pipe.

=cut

sub open ($)
{
  my ($self, $file) = @_;

  verbose 3, "opening $file";

  $self->SUPER::open($file)
    or die "$me: cannot open $file: $!\n";
}

=item C<close>

=cut

sub close
{
  my $this = shift;
  $this->SUPER::close (@_)
    or die "$me: cannot close $this:$!\n";
}

=item C<line>

Return the next line, chomped.

=cut

sub line
{
  my $this = shift;
  my $res = $this->SUPER::getline (@_);
  chomp $res
    if $res;
  $res;
}

=item C<lines>

Return the remaining lines (chomped) as an array.

=cut

sub lines
{
  my $this = shift;
  my @res = $this->SUPER::getlines (@_);
  map { chomp($_) } @res;
  @res;
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
