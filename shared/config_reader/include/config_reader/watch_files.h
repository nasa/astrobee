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

#ifndef CONFIG_READER_WATCH_FILES_H_
#define CONFIG_READER_WATCH_FILES_H_

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

namespace config_reader {

class ActiveFile{
 public:
  std::string name;
 protected:
  time_t mod_time;
 protected:
  time_t getFileModTime();
 public:
  ActiveFile()
    {mod_time = 0;}

  void init(const char *_name)
    {name = _name; mod_time = 0;}
  const char *operator()()
    {return(name.c_str());}

  void invalidate()
    {mod_time = 0;}
  bool isModified()
    {return(mod_time != getFileModTime());}
  void markAsRead()
    {mod_time = getFileModTime();}
};

class WatchFiles{
 protected:
  static const uint32_t ModEvents = IN_CLOSE_WRITE|IN_DELETE_SELF;

 public:
  class Watch{
   protected:
    WatchFiles *parent;
    int wd;  // watch descriptor
   public:
    Watch()
      {parent = NULL; wd = -1;}
    Watch(const Watch &w)
      {parent = w.parent; wd = w.wd;}

    bool valid()
      {return(parent != NULL && wd >= 0);}
    bool watch(WatchFiles *_parent, const char *filename)
      {return(_parent && _parent->addWatch(this, filename));}
    bool rewatch(const char *filename)
      {return(parent && parent->addWatch(this, filename));}
    bool remove()
      {return(parent && parent->removeWatch(this));}

    bool isFileModified()
      {return(parent && (parent->calcEventMask(this) & ModEvents));}

    friend class WatchFiles;
  };

 protected:
  std::vector<inotify_event> events;
  int inotify_fd;
  int num_watches;

 public:
  WatchFiles()
    {inotify_fd = -1; num_watches = 0;}
  ~WatchFiles()
    {reset();}

  bool init();
  void reset();
  bool isInited()
    {return(inotify_fd >= 0);}

  bool addWatch(Watch *w, const char *filename);
  bool removeWatch(Watch *w);
  uint32_t calcEventMask(Watch *w);

  int getEvents();
  int getNumEvents()
    {return(events.size());}
  void clearEvents()
    {events.clear();}
};

time_t FileModTime(const char *filename);
void SetNonBlocking(int fd);

}  // namespace config_reader

#endif  // CONFIG_READER_WATCH_FILES_H_
