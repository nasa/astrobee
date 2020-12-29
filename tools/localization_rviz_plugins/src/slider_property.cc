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
#include "slider_property.h" // NOLINT

namespace rviz {
SliderProperty::SliderProperty(const QString& name, const int default_value, const QString& description,
                               Property* parent, const char* changed_slot, QObject* receiver)
    : Property(name, default_value, description, parent, changed_slot, receiver), max_(0), min_(0) {}

QWidget* SliderProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& /*option*/) {
  slider_ = new QSlider(Qt::Horizontal, parent);
  slider_->setMaximum(max_);
  slider_->setMinimum(min_);
  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
  return slider_;
}

void SliderProperty::setValue(const int value) { slider_->setValue(value); }

void SliderProperty::setMaximum(const int max) { max_ = max; }

void SliderProperty::setMinimum(const int min) { min_ = min; }

int SliderProperty::getInt() const {
  // TODO(rsoussan): Better way to handle this? Create slider in constructor?
  if (slider_ == nullptr) return 0;
  return getValue().toInt();
}
}  // end namespace rviz
