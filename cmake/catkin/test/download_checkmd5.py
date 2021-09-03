#!/usr/bin/env python

from __future__ import print_function
import errno
import os
import sys
try:
    from urllib.request import addinfourl, BaseHandler, build_opener, Request, URLError
except ImportError:
    from urllib2 import addinfourl, BaseHandler, build_opener, Request, URLError
import hashlib
from argparse import ArgumentParser

NAME = "download_checkmd5.py"


class HTTPRangeHandler(BaseHandler):

    def http_error_206(self, req, fp, code, msg, hdrs):
        r = addinfourl(fp, hdrs, req.get_full_url())
        r.code = code
        r.msg = msg
        return r

    def http_error_416(self, req, fp, code, msg, hdrs):
        raise URLError('Requested Range Not Satisfiable')


def download_with_resume(uri, dest):
    handler = HTTPRangeHandler()
    opener = build_opener(handler)

    offset = 0
    content_length = None
    accept_ranges = False
    while True:
        req = Request(uri)
        if offset:
            req.add_header('Range', 'bytes=%d-' % offset)
        src_file = None
        try:
            src_file = opener.open(req)
            headers = src_file.info()
            if not offset:
                # on first connection check server capabilities
                if 'Content-Length' in headers:
                    content_length = int(headers['Content-Length'])
                if 'Accept-Ranges' in headers:
                    accept_ranges = headers['Accept-Ranges'] != 'none'
            else:
                # on resume verify that server understood range header and responded accordingly
                if 'Content-Range' not in headers:
                    raise IOError('Download aborted and server does not support resuming download')
                if int(headers['Content-Range'][len('bytes '):].split('-')[0]) != offset:
                    raise IOError('Download aborted because server replied with different content range then requested')
                sys.stdout.write(' resume from %d...' % offset)
                sys.stdout.flush()
            with open(dest, 'ab' if offset else 'wb') as dst_file:
                progress = False
                while True:
                    data = src_file.read(8192)
                    if not data:
                        break
                    progress = True
                    dst_file.write(data)
                    offset += len(data)
                if not progress:
                    # if no bytes have been received abort download
                    raise IOError("No progress when trying to download '%s'" % uri)
        except:
            if src_file:
                src_file.close()
            raise

        # when content length is unknown it is assumed that the download is complete
        if content_length is None:
            break
        # or when enough data has been downloaded (> is especially a valid case)
        if offset >= content_length:
            break
        if not accept_ranges:
            raise IOError('Server does not accept ranges to resume download')


def download_md5(uri, dest):
    """
    downloads file from uri to file dest
    """
    # Create intermediate directories as necessary, #2970
    dirname = os.path.dirname(dest)
    if len(dirname):
        try:
            os.makedirs(dirname)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

    sys.stdout.write('Downloading %s to %s...' % (uri, dest))
    sys.stdout.flush()
    try:
        download_with_resume(uri, dest)
        sys.stdout.write(' done.\n')
    except Exception as e:
        # delete partially downloaded data
        if os.path.exists(dest):
            os.unlink(dest)
        sys.stdout.write(' failed (%s)!\n' % e)
        raise


def checkmd5(dest, md5sum=None):
    """
    checks file at dest against md5.
    :returns (boolean, hexdigest): True if dest contents matches md5sum
    """
    if not os.path.exists(dest):
        return False, 'null'
    with open(dest, 'rb') as f:
        md5value = hashlib.md5()
        while True:
            buf = f.read(4096)
            if not buf:
                break
            md5value.update(buf)
    hexdigest = md5value.hexdigest()

    print('Checking md5sum on %s' % (dest))
    return hexdigest == md5sum, hexdigest


def main(argv=sys.argv[1:]):
    """
    Dowloads URI to file dest and checks md5 if given.
    """
    parser = ArgumentParser(description='Dowloads URI to file dest. If md5sum is given, checks md5sum. If file existed and mismatch, downloads and checks again')
    parser.add_argument('uri')
    parser.add_argument('dest')
    parser.add_argument('md5sum', nargs='?')
    parser.add_argument('--ignore-error', action='store_true', help='Ignore download errors')
    args = parser.parse_args(argv)

    uri = args.uri
    if '://' not in uri:
        uri = 'file://' + uri

    fresh = False
    if not os.path.exists(args.dest):
        try:
            download_md5(uri, args.dest)
        except Exception:
            if args.ignore_error:
                return 0
            raise
        fresh = True

    if args.md5sum:
        result, hexdigest = checkmd5(args.dest, args.md5sum)
        if result is False and fresh is False:
            print('WARNING: md5sum mismatch (%s != %s); re-downloading file %s' % (hexdigest, args.md5sum, args.dest))
            os.remove(args.dest)
            try:
                download_md5(uri, args.dest)
            except Exception:
                if args.ignore_error:
                    return 0
                raise
            result, hexdigest = checkmd5(args.dest, args.md5sum)
        if result is False:
            return 'ERROR: md5sum mismatch (%s != %s) on %s; aborting' % (hexdigest, args.md5sum, args.dest)

    return 0


if __name__ == '__main__':
    sys.exit(main())
