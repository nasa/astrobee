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
#ifndef LOCALIZATION_RVIZ_PLUGINS_SLIDER_PROPERTY_H  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_SLIDER_PROPERTY_H  // NOLINT

#include <rviz/properties/property.h>

#include <QSlider>

namespace rviz {
class SliderProperty : public Property {
  Q_OBJECT
 public:
  SliderProperty(const QString& name = QString(), const int default_value = 0, const QString& description = QString(),
                 Property* parent = nullptr, const char* changed_slot = nullptr, QObject* receiver = nullptr);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override;

  void setMaximum(const int max);

  void setMinimum(const int min);

  int getInt() const;

 public Q_SLOTS:  // NOLINT
  void setValue(const int value);

 Q_SIGNALS:

 protected:
  QSlider* slider_;
  int max_;
  int min_;
  int value_;
};

}  // end namespace rviz

#endif  // LOCALIZATION_RVIZ_PLUGINS_SLIDER_PROPERTY_H // NOLINT
