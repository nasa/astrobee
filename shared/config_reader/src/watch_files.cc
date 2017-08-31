/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// Code taken from www.cs.cmu.edu/~coral/projects/localization/source.html
// Brian Coltin is in the team and said this code could be used for the project

#include "config_reader/watch_files.h"


/*
  [ References ]

  Inofity Documentation and FAQ
    http://inotify.aiken.cz/?section=inotify&page=doc&lang=en
    http://inotify.aiken.cz/?section=inotify&page=faq&lang=en
*/

//====================================================================//

time_t config_reader::FileModTime(const char *filename) {
  struct stat st;
  st.st_mtime = 0;
  stat(filename, &st);
  return(st.st_mtime);
}

void config_reader::SetNonBlocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) flags = 0;
  fcntl(fd, F_SETFL, flags|O_NONBLOCK);
}

//====================================================================//

time_t config_reader::ActiveFile::getFileModTime() {
  return(FileModTime(name.c_str()));
}

//====================================================================//

bool config_reader::WatchFiles::init() {
  reset();

  inotify_fd = inotify_init();
  if (inotify_fd < 0) return(false);
  SetNonBlocking(inotify_fd);

  return(true);
}

void config_reader::WatchFiles::reset() {
  if (isInited()) {
    close(inotify_fd);
    inotify_fd = -1;
  }
  events.clear();
  num_watches = 0;
}

bool config_reader::WatchFiles::addWatch(Watch *w, const char *filename) {
  // initialize if needed
  if (!isInited() && !init()) return(false);

  // remove existing watch if present
  if (w->parent) removeWatch(w);

  // add watch for file
  w->wd = inotify_add_watch(inotify_fd, filename, ModEvents);
  if (w->wd < 0) return(false);
  w->parent = this;
  num_watches++;

  return(true);
}

bool config_reader::WatchFiles::removeWatch(Watch *w) {
  if (w->parent != this) return(false);

  int ret = inotify_rm_watch(inotify_fd, w->wd);
  w->parent = NULL;
  w->wd = -1;
  if (ret == 0) num_watches--;

  return(ret == 0);
}

uint32_t config_reader::WatchFiles::calcEventMask(Watch *w) {
  uint32_t mask = 0;
  for (unsigned i = 0; i < events.size(); i++) {
    if (events[i].wd == w->wd) mask |= events[i].mask;
  }
  return(mask);
}

int config_reader::WatchFiles::getEvents() {
  static const int BufSize = 512;
  char buf[BufSize];

  while (true) {
    // get a buffer of events
    int nr = read(inotify_fd, buf, BufSize);
    if (nr < static_cast<int>(sizeof(inotify_event))) break;

    // add them to our interal event queue
    int i = 0;
    while (i < nr) {
      const inotify_event &e = *reinterpret_cast<inotify_event*>(buf+i);
      events.push_back(e);
      i += sizeof(inotify_event) + e.len;
    }
  }

  return(events.size());
}
