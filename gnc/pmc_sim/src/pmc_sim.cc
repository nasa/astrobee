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

#include <ros/console.h>
#include "pmc_sim/pmc_sim.h"

namespace pmc_sim {

const int Blower::LOOKUP_TABLE_SIZE = 334;
const float Blower::AREA_LOOKUP_INPUT[334] =
  { 0.0F, 1.46138646E-5F, 2.92062887E-5F, 4.37784511E-5F, 5.83315159E-5F,
    7.28665837E-5F, 8.7384753E-5F, 0.000101887068F, 0.000116374569F,
    0.000130848246F, 0.000145309066F, 0.000159758F, 0.000174195899F,
    0.000188623759F, 0.000203042364F, 0.000217452631F, 0.000231855331F,
    0.000246251322F, 0.000260641362F, 0.000275026192F, 0.000289406598F,
    0.000303783338F, 0.00031815705F, 0.000332528463F, 0.000346898247F,
    0.000361267099F, 0.00037563566F, 0.000390004541F, 0.000404374383F,
    0.000418745767F, 0.000433119305F, 0.000447495579F, 0.000461875083F,
    0.000476258458F, 0.000490646285F, 0.000505038945F, 0.00051943725F,
    0.000533841376F, 0.000548252079F, 0.000562669651F, 0.00057709479F,
    0.000591527845F, 0.000605969399F, 0.000620419742F, 0.000634879572F,
    0.000649349065F, 0.000663828861F, 0.000678319368F, 0.000692821F,
    0.000707334315F, 0.000721859455F, 0.000736397109F, 0.000750947569F,
    0.0007655113F, 0.000780088594F, 0.000794680149F, 0.000809286F,
    0.000823906856F, 0.000838542939F, 0.000853194739F, 0.000867862604F,
    0.000882546941F, 0.000897248159F, 0.00091196649F, 0.000926702633F,
    0.000941456819F, 0.0009562294F, 0.00097102084F, 0.000985831604F,
    0.00100066199F, 0.00101551227F, 0.00103038305F, 0.0010452749F,
    0.00106018782F, 0.00107512227F, 0.00109007885F, 0.00110505789F,
    0.00112005987F, 0.00113508501F, 0.00115013402F, 0.00116520724F,
    0.0011803048F, 0.00119542738F, 0.00121057557F, 0.00122574961F, 0.00124095F,
    0.00125617709F, 0.00127143157F, 0.00128671364F, 0.001302024F, 0.00131736277F,
    0.00133273099F, 0.00134812866F, 0.00136355648F, 0.00137901504F,
    0.00139450456F, 0.00141002587F, 0.00142557954F, 0.0014411658F,
    0.00145678537F, 0.00147243857F, 0.00148812647F, 0.00150384917F,
    0.00151960761F, 0.00153540191F, 0.00155123312F, 0.0015671018F,
    0.00158300844F, 0.00159895339F, 0.00161493802F, 0.00163096236F,
    0.00164702744F, 0.0016631335F, 0.00167928194F, 0.00169547275F,
    0.00171170721F, 0.00172798557F, 0.00174430886F, 0.0017606779F,
    0.00177709304F, 0.00179355568F, 0.00181006629F, 0.00182662567F,
    0.00184323464F, 0.00185989426F, 0.00187660486F, 0.0018933682F,
    0.00191018439F, 0.00192705484F, 0.00194398011F, 0.00196096138F,
    0.00197799969F, 0.00199509552F, 0.00201225071F, 0.00202946551F,
    0.00204674155F, 0.00206407951F, 0.00208148058F, 0.00209894567F,
    0.00211647665F, 0.00213407329F, 0.00215173839F, 0.00216947193F,
    0.00218727579F, 0.00220515113F, 0.00222309888F, 0.00224112067F,
    0.0022592172F, 0.0022773908F, 0.00229564216F, 0.00231397292F, 0.00233238423F,
    0.00235087774F, 0.00236945506F, 0.00238811737F, 0.00240686629F,
    0.00242570369F, 0.00244463095F, 0.00246364973F, 0.00248276163F,
    0.00250196829F, 0.00252127182F, 0.00254067336F, 0.00256017526F,
    0.00257977913F, 0.00259948708F, 0.00261930074F, 0.00263922219F,
    0.00265925354F, 0.00267939596F, 0.00269965269F, 0.00272002514F,
    0.00274051586F, 0.00276112719F, 0.002781861F, 0.00280271959F, 0.00282370578F,
    0.00284482073F, 0.00286606839F, 0.00288745062F, 0.00290896976F,
    0.00293062883F, 0.00295243063F, 0.00297437701F, 0.00299647171F,
    0.00301871705F, 0.00304111606F, 0.00306367176F, 0.00308638718F,
    0.00310926535F, 0.0031323093F, 0.00315552298F, 0.00317890849F,
    0.00320247072F, 0.00322621223F, 0.00325013627F, 0.00327424728F,
    0.00329854828F, 0.00332304346F, 0.00334773702F, 0.0033726322F,
    0.00339773367F, 0.00342304516F, 0.00344857131F, 0.00347431679F,
    0.00350028556F, 0.00352648227F, 0.00355291134F, 0.00357957813F,
    0.00360648776F, 0.0036336449F, 0.00366105489F, 0.00368872308F,
    0.00371665484F, 0.00374485622F, 0.00377333187F, 0.00380208902F,
    0.00383113371F, 0.00386047084F, 0.00389010808F, 0.00392005173F,
    0.00395030761F, 0.00398088479F, 0.00401178841F, 0.0040430259F,
    0.00407460658F, 0.00410653651F, 0.00413882406F, 0.0041714781F,
    0.00420450559F, 0.00423791679F, 0.00427172F, 0.00430592475F, 0.00434054F,
    0.00437557558F, 0.00441104267F, 0.00444695121F, 0.00448331F, 0.00452013267F,
    0.00455742935F, 0.00459521171F, 0.00463349326F, 0.00467228517F,
    0.00471160142F, 0.00475145364F, 0.00479185674F, 0.00483282562F,
    0.00487437518F, 0.00491652032F, 0.00495927641F, 0.00500265928F,
    0.00504668895F, 0.00509137847F, 0.00513675157F, 0.0051828213F,
    0.00522961235F, 0.00527714146F, 0.00532543147F, 0.00537450658F,
    0.00542438496F, 0.00547509314F, 0.00552665396F, 0.0055790972F,
    0.00563244335F, 0.00568672409F, 0.00574196922F, 0.00579820713F,
    0.00585547043F, 0.00591378892F, 0.00597320078F, 0.00603374047F,
    0.00609544525F, 0.00615835097F, 0.00622250512F, 0.0062879459F, 0.006354718F,
    0.00642287126F, 0.00649245409F, 0.00656351773F, 0.00663611386F,
    0.00671030255F, 0.00678614574F, 0.00686370581F, 0.00694304705F,
    0.00702424115F, 0.00710735889F, 0.00719248503F, 0.0072797006F,
    0.00736908847F, 0.00746074459F, 0.00755477371F, 0.00765126897F,
    0.00775034819F, 0.00785212312F, 0.00795672741F, 0.00806429F, 0.00817495212F,
    0.00828887522F, 0.00840621162F, 0.00852714293F, 0.00865184888F,
    0.00878054276F, 0.00891342945F, 0.00905075204F, 0.00919276662F,
    0.0093397228F, 0.00949195493F, 0.00964974798F, 0.00981347449F,
    0.00998349674F, 0.0101602506F, 0.0103441793F, 0.0105357962F, 0.010735645F,
    0.0109443273F, 0.0111625558F, 0.0113910241F, 0.0116305985F, 0.0118822251F,
    0.0121469265F, 0.0124259172F, 0.0127205F, 0.0130322482F, 0.0133628659F,
    0.0137143685F, 0.0140890917F, 0.0144897308F, 0.0149194878F, 0.01538203F,
    0.0158818662F, 0.0164243504F, 0.0170160942F, 0.0176650118F, 0.0183811728F };
const float Blower::CDP_LOOKUP_OUTPUT[334] =
  { 0.0825F, 0.082625635F, 0.08274699F, 0.0828641355F, 0.082977131F,
    0.083086051F, 0.0831909552F, 0.083291918F, 0.083389F, 0.083482258F,
    0.0835717544F, 0.0836575627F, 0.0837397277F, 0.083818309F, 0.0838933736F,
    0.0839649737F, 0.0840331689F, 0.0840980038F, 0.0841595381F, 0.0842178315F,
    0.0842729211F, 0.084324874F, 0.0843737274F, 0.0844195336F, 0.0844623446F,
    0.0845022127F, 0.0845391676F, 0.0845732689F, 0.0846045539F, 0.0846330673F,
    0.0846588612F, 0.0846819654F, 0.0847024247F, 0.0847202763F, 0.0847355723F,
    0.0847483352F, 0.0847586095F, 0.0847664326F, 0.0847718418F, 0.0847748667F,
    0.0847755447F, 0.084773913F, 0.08477F, 0.0847638398F, 0.0847554579F,
    0.084744893F, 0.0847321674F, 0.0847173184F, 0.0847003683F, 0.0846813396F,
    0.0846602693F, 0.0846371725F, 0.0846120864F, 0.0845850259F, 0.0845560208F,
    0.0845250934F, 0.0844922587F, 0.0844575465F, 0.0844209716F, 0.0843825564F,
    0.0843423232F, 0.0843002871F, 0.0842564702F, 0.084210895F, 0.0841635615F,
    0.0841145F, 0.0840637162F, 0.0840112418F, 0.0839570761F, 0.083901234F,
    0.0838437378F, 0.0837845877F, 0.0837238058F, 0.0836614F, 0.0835973844F,
    0.0835317597F, 0.083464548F, 0.0833957493F, 0.0833253786F, 0.0832534432F,
    0.0831799433F, 0.0831048936F, 0.0830283F, 0.0829501674F, 0.0828705F,
    0.0827893F, 0.0827065781F, 0.0826223418F, 0.0825365856F, 0.0824493095F,
    0.0823605284F, 0.0822702423F, 0.0821784437F, 0.08208514F, 0.0819903314F,
    0.0818940252F, 0.0817962065F, 0.0816968903F, 0.081596069F, 0.0814937353F,
    0.081389904F, 0.0812845528F, 0.0811777F, 0.0810693204F, 0.0809594318F,
    0.0808480158F, 0.0807350799F, 0.0806206167F, 0.0805046186F, 0.0803870782F,
    0.080268F, 0.0801473707F, 0.0800251886F, 0.0799014494F, 0.0797761381F,
    0.0796492621F, 0.0795208F, 0.0793907568F, 0.0792591125F, 0.0791258737F,
    0.0789910182F, 0.0788545534F, 0.0787164569F, 0.0785767287F, 0.0784353614F,
    0.0782923326F, 0.0781476498F, 0.0780012906F, 0.077853255F, 0.0777035281F,
    0.0775521F, 0.0773989633F, 0.0772441104F, 0.0770875216F, 0.0769291893F,
    0.0767691135F, 0.0766072646F, 0.0764436498F, 0.0762782469F, 0.076111041F,
    0.0759420395F, 0.0757712126F, 0.0755985528F, 0.0754240528F, 0.0752477F,
    0.0750694722F, 0.0748893768F, 0.0747073889F, 0.0745235F, 0.0743377F,
    0.0741499662F, 0.0739603F, 0.0737686828F, 0.0735751F, 0.0733795539F,
    0.0731820166F, 0.0729824826F, 0.072780937F, 0.0725773647F, 0.072371766F,
    0.0721641108F, 0.0719544F, 0.071742624F, 0.0715287626F, 0.0713128075F,
    0.0710947439F, 0.0708745569F, 0.0706522539F, 0.0704278F, 0.0702012F,
    0.0699724406F, 0.0697415F, 0.0695083737F, 0.0692730546F, 0.0690355226F,
    0.0687957704F, 0.0685537905F, 0.0683095828F, 0.0680631176F, 0.0678143948F,
    0.0675634F, 0.0673101321F, 0.0670545697F, 0.0667967126F, 0.0665365383F,
    0.0662740618F, 0.0660092533F, 0.0657421201F, 0.0654726326F, 0.0652008057F,
    0.0649266243F, 0.0646500662F, 0.0643711537F, 0.0640898496F, 0.0638061538F,
    0.0635200813F, 0.0632316F, 0.0629407242F, 0.0626474321F, 0.0623517223F,
    0.0620536F, 0.0617530458F, 0.0614500828F, 0.0611446686F, 0.0608368181F,
    0.0605265312F, 0.0602138F, 0.0598986298F, 0.0595810153F, 0.0592609458F,
    0.0589384325F, 0.0586134642F, 0.0582860447F, 0.0579561703F, 0.0576238409F,
    0.0572890751F, 0.0569518507F, 0.0566121712F, 0.0562700555F, 0.0559254885F,
    0.0555784889F, 0.0552290566F, 0.054877162F, 0.0545228682F, 0.0541661382F,
    0.0538069829F, 0.0534454137F, 0.0530814491F, 0.0527150668F, 0.0523463078F,
    0.0519751534F, 0.0516016223F, 0.0512257218F, 0.0508474559F, 0.0504668579F,
    0.0500839F, 0.0496986173F, 0.0493110269F, 0.0489211343F, 0.0485289469F,
    0.0481344871F, 0.0477377549F, 0.04733878F, 0.0469375588F, 0.0465341397F,
    0.0461285152F, 0.0457207039F, 0.0453107134F, 0.0448985882F, 0.0444843248F,
    0.0440679714F, 0.0436495095F, 0.0432290025F, 0.0428064205F, 0.0423818417F,
    0.0419552475F, 0.0415266976F, 0.0410961919F, 0.0406637266F, 0.0402293913F,
    0.0397931822F, 0.0393551365F, 0.038915243F, 0.0384735875F, 0.0380301885F,
    0.0375850387F, 0.0371381938F, 0.0366896801F, 0.0362395607F, 0.0357878208F,
    0.0353345275F, 0.0348797F, 0.0344234072F, 0.0339656323F, 0.033506453F,
    0.033045914F, 0.032584019F, 0.0321208276F, 0.0316563807F, 0.0311907455F,
    0.0307239257F, 0.0302559771F, 0.0297869369F, 0.0293168686F, 0.0288458094F,
    0.0283738412F, 0.0279009566F, 0.0274272151F, 0.0269527063F, 0.0264774524F,
    0.0260014758F, 0.0255248956F, 0.0250477232F, 0.0245700441F, 0.0240918733F,
    0.0236133039F, 0.0231343843F, 0.0226551369F, 0.0221756734F, 0.0216960311F,
    0.0212163F, 0.0207364894F, 0.0202567242F, 0.0197770186F, 0.0192974396F,
    0.0188181363F, 0.0183390342F, 0.0178603418F, 0.0173820443F, 0.0169042759F,
    0.0164270438F, 0.0159504749F, 0.0154745989F, 0.01499952F, 0.0145253353F,
    0.0140520222F, 0.0135798045F, 0.0131086893F, 0.0126387365F, 0.0121700801F,
    0.0117027499F, 0.0112369023F, 0.0107725374F, 0.0103098117F, 0.0098488F,
    0.00938956812F, 0.00893222168F, 0.00847681239F, 0.00802352652F,
    0.00757240178F, 0.007123549F, 0.00667699846F, 0.00623294385F, 0.0057914448F};

void PID::Initialize(float Kp, float Ki, float Kd, float out_max, float out_min) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
  omax = out_max;
  omin = out_min;
  last = integral = 0.0;
}

float PID::Run(float x) {
  float dt = 1.0 / 62.5;
  float d = kd * x - last;
  float out = kp * x + ki * integral + d;
  last += d * dt;  // kind of confused here but matches simmulink
  integral += x * dt;
  if (out < omin)
    out = omin;
  if (out > omax)
    out = omax;
  return out;
}

Blower::Blower(bool is_pmc1) {
  prev_omega_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  backlash_theta_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  motor_thetas_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  prev_speed_rate_ = impeller_speed_ = 0.0f;
  for (int i = 0; i < 6; i++)
    servo_pids_[i].Initialize(5.0, 1.5, 0.8, 6.0, -6.0);
  speed_pid_.Initialize(0.2, 0.1, 0.05, 16.6, -16.6);
  if (is_pmc1) {
    // determined by test
    discharge_coeff_ << 0.914971062, 0.755778254, 0.940762925, 0.792109779, 0.92401881, 0.930319765;
    zero_thrust_area_ = 0.0044667;  //  [m^2] If air is leaking from the plenum, the
    // '0 thrust' position (all nozzles closed) will have an effective open area
    nozzle_offsets_    << 6.00,  4.01, -1.56,
                         -6.00,  4.01,  1.56,
                          2.83,  6.00,  2.83,
                         -2.83,  6.00, -2.83,
                         -2.66,  4.01,  6.00,
                          2.66,  4.01, -6.00;
    nozzle_orientations_ << 1, 0,  0,  //  [unit vec] PM1: Pointing Direction of each nozzle
                           -1, 0,  0,
                            0, 1,  0,
                            0, 1,  0,
                            0, 0,  1,
                            0, 0, -1;
    impeller_orientation_ << 0, 1, 0;
  } else {
    discharge_coeff_ << 0.947114008, 0.764468916, 1.000000000, 0.90480943, 0.936555627, 0.893794766;
    zero_thrust_area_ = 0.0042273;
    nozzle_offsets_  << -6.00, -4.01, -1.56,
                         6.00, -4.01,  1.56,
                        -2.83, -6.00,  2.83,
                         2.83, -6.00, -2.83,
                         2.66, -4.01,  6.00,
                        -2.66, -4.01, -6.00;
    nozzle_orientations_ << -1, 0,  0,  //  [unit vec] PM1: Pointing Direction of each nozzle
                             1, 0,  0,
                             0, -1, 0,
                             0, -1, 0,
                             0, 0,  1,
                             0, 0, -1;
    impeller_orientation_ << 0, -1, 0;
  }
  const float units_inches_to_meters = 0.0254;
  nozzle_offsets_ *= units_inches_to_meters;  // [m] Position vector of the nozzle locations in the body frame

  center_of_mass_ << 0.0, 0.0, 0.0;  // hmmm.... this seems wrong but was used originally in the simulink
}

void Blower::ServoModel(const Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Matrix<float, 6, 1> & nozzle_theta,
                        Eigen::Matrix<float, 6, 1> & nozzle_area) {
  //  [rad]      THEORETICAL max angle that a nozzle can open to
  const float abp_nozzle_max_open_angle = 79.91 * M_PI / 180.0;
  //  [rad]      Min angle that the nozzle can close to
  const float abp_nozzle_min_open_angle = 15.68 * M_PI / 180.0;
  const float abp_nozzle_flap_length = 0.5353 * 0.0254;  //  [m]
  const float abp_nozzle_intake_height = 0.5154 * 0.0254;  //  [m]
  const float abp_nozzle_gear_ratio = 0.5;
  const float bpm_servo_max_theta = abp_nozzle_gear_ratio * (abp_nozzle_max_open_angle - abp_nozzle_min_open_angle);
  //  [-] Conversion between servo PWM command (0-100) and resulting angle (0-90)
  const float bpm_servo_pwm2angle = bpm_servo_max_theta / 255.0;
  // bpm_servo_peak_torque*bpm_servo_motor_gear_ratio/(bpm_servo_peak_curr); [Nm/A] Servo motor torque constant
  const float motor_k = 0.2893 * 0.01 / 1.5;
  // bpm_servo_max_voltage/bpm_servo_peak_curr; %[ohm] Servo internal resistance
  const float motor_r = 6.0 / 1.5;
  // bpm_imp_noload_curr*bpm_imp_motor_torque_k/bpm_imp_noload_speed ;  %[Nm*s] Viscous Friction
  const float motor_friction = 3e-7;
  //  %[kg*m^2] (Tuned in analysis_servo_motor_test.m) Servo gearbox inertia.
  const float motor_gearbox_inertia = .000000025;
  const float motor_gear_ratio = 1.0 / 100.0;
  const float backlash_half_width = 1.0 * M_PI / 180.0 / 2;
  Eigen::Matrix<float, 6, 1> nozzle_widths;
  nozzle_widths << 5.0, 5.0, 2.8, 2.8, 2.8, 2.8;
  nozzle_widths *= 0.0254;  //  [m]
  // backlash box
  for (int i = 0; i < 6; i++) {
    if (motor_thetas_[i] < backlash_theta_[i] - backlash_half_width)
      backlash_theta_[i] = motor_thetas_[i] + backlash_half_width;
    else if (motor_thetas_[i] > backlash_theta_[i] + backlash_half_width)
      backlash_theta_[i] = motor_thetas_[i] - backlash_half_width;
  }
  Eigen::Matrix<float, 6, 1> voltage, err = bpm_servo_pwm2angle * servo_cmd - backlash_theta_ * motor_gear_ratio;
  for (int i = 0; i < 6; i++)
    voltage[i] = servo_pids_[i].Run(err[i]);
  servo_current_ = (voltage - motor_k * prev_omega_) * (1.0 / motor_r);
  motor_thetas_ += 1.0 / 62.5 * prev_omega_;
  motor_thetas_ = motor_thetas_.cwiseMin(bpm_servo_max_theta / motor_gear_ratio).cwiseMax(0.0);
  // torque - viscous friction
  Eigen::Matrix<float, 6, 1> motor_torque = motor_k * servo_current_ - motor_friction * prev_omega_;
  prev_omega_ += (1.0 / motor_gearbox_inertia) / 62.5 * motor_torque;
  nozzle_theta = backlash_theta_ * motor_gear_ratio * 1.0 / abp_nozzle_gear_ratio;
  for (int i = 0; i < 6; i++) {
    //  * nozzle_flap_count * nozzle_width
    nozzle_area[i] = (abp_nozzle_intake_height - cos(abp_nozzle_min_open_angle + nozzle_theta[i]) *
                      abp_nozzle_flap_length) * 2 * nozzle_widths[i];
  }
}

void Blower::ImpellerModel(int speed_cmd, float voltage, float* motor_current, float* motor_torque) {
  const float impeller_speed2pwm = 0.792095;  //  [CNT/(rad/sec)] Converts impeller speeds into PWM values
  const float max_change = 200*2*M_PI/60.0 / 62.5;
  float speed = speed_cmd / impeller_speed2pwm;
  if (speed > prev_speed_rate_ + max_change)
    speed = prev_speed_rate_ + max_change;
  else if (speed < prev_speed_rate_ - max_change)
    speed = prev_speed_rate_ - max_change;
  prev_speed_rate_ = speed;

  speed = speed - impeller_speed_;
  float v = speed_pid_.Run(speed);
  v = std::max(-voltage, std::min(voltage, v));

  // dc_motor_model
  const float motor_speed_k = 374*2*M_PI/60;  // [(rad/s)/V] Impeller Speed constant
  const float motor_r = 1.2;  // %[ohm] Impeller Motor internal resistance
  const float motor_torque_k = 0.0255;  // [Nm/A] Impeller Torque constant
  // noload_curr * torque_k / noload_speed [Nm*s] Viscous Friction
  const float motor_friction_coeff = 0.144 * motor_torque_k / (4380*2*M_PI/60.0);
  *motor_current = (v - impeller_speed_ / motor_speed_k) / motor_r;
  *motor_torque = (*motor_current) * motor_torque_k - motor_friction_coeff * impeller_speed_;

  last_speed_ = impeller_speed_;  // matching simulink, but I don't think this is right
  // in simulink, drag torque would be subtracted from motor, but set to 0
  // noise is also set to 0
  const float impeller_inertia = 0.001;
  impeller_speed_ += *motor_torque / impeller_inertia / 62.5;
}

float Blower::Lookup(int table_size, const float lookup[], const float breakpoints[], float value) {
  if (value < breakpoints[0])
    return lookup[0];
  if (value > breakpoints[table_size - 1])
    return lookup[table_size - 1];
  int start = 0, end = table_size;
  while (start < end - 1) {
    int mid = (start + end) / 2;
    if (breakpoints[mid] <= value)
      start = mid;
    else
      end = mid;
  }

  float ratio = (value - breakpoints[start]) / (breakpoints[start + 1] - breakpoints[start]);
  return (1.0 - ratio) * lookup[start] + ratio * lookup[start + 1];
}

Eigen::Matrix<float, 6, 1> Blower::Aerodynamics(const Eigen::Matrix<float, 6, 1> & nozzle_area) {
  float area = zero_thrust_area_ + (discharge_coeff_.array() * nozzle_area.array()).sum();
  float cdp = Lookup(LOOKUP_TABLE_SIZE, CDP_LOOKUP_OUTPUT, AREA_LOOKUP_INPUT, area);
  const float impeller_diameter = 5.5 * 0.0254;  // [m]
  const float air_density = 1.2;  //  [kg/m^3]   Air density inside of the ISS (some places in code this was 1.225?)
  float delta_p = impeller_speed_ * impeller_speed_ * cdp * impeller_diameter * impeller_diameter * air_density;
  // 1.25 is thrust_error_scale_factor
  return 2 * delta_p * discharge_coeff_.array() * discharge_coeff_.array() * nozzle_area.array() * 1.25;
  // note: currently in simulink noise is set to zero
}

void Blower::Step(int impeller_cmd, Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Vector3f & omega, float voltage) {
  Eigen::Matrix<float, 6, 1> nozzle_area;
  float motor_torque;
  ServoModel(servo_cmd, nozzles_theta_, nozzle_area);
  ImpellerModel(impeller_cmd, voltage, &impeller_current_, &motor_torque);
  Eigen::Matrix<float, 6, 1> nozzle_thrust = Aerodynamics(nozzle_area);
  BlowerBodyDynamics(omega, nozzle_thrust, motor_torque);
}

void Blower::BlowerBodyDynamics(const Eigen::Vector3f & omega_body, Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
    float motor_torque) {
  Eigen::Matrix<float, 3, 6> thrust2force, thrust2torque;
  // latch nozzle thrust matrices
  Eigen::Matrix<float, 6, 3> nozzle_moment_arm = nozzle_offsets_ - center_of_mass_.transpose().replicate(6, 1);
  for (int i = 0; i < 6; i++)
    thrust2torque.col(i) = -nozzle_moment_arm.row(i).cross(nozzle_orientations_.row(i)).transpose();

  thrust2force = -nozzle_orientations_.transpose();

  const float impeller_inertia = 0.001;
  Eigen::Vector3f momentum_B = impeller_orientation_ * impeller_inertia * impeller_speed_;

  Eigen::Matrix<float, 6, 1> a = nozzle_thrusts;
  force_B_ = thrust2force * nozzle_thrusts;
  torque_B_ = impeller_orientation_ * -motor_torque + omega_body.cross(momentum_B) + thrust2torque * nozzle_thrusts;
}

PMCSim::PMCSim(void) : b1(true), b2(false), omega_(0, 0, 0), voltage_(0.0f) {
  for (int i = 0; i < 2; i++) {
    impeller_cmd_[i] = 0;
    servo_cmd_[i] << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  }
  gnc_.Initialize();
}

void TestTwoVectors(const char* name, const Eigen::VectorXf new_array,
                    const float old_array[], float tolerance) {
  for (int i = 0; i < new_array.rows(); i++) {
    float difference = old_array[i] - new_array[i];
    float perc_difference = difference / old_array[i];
    if (fabs(difference) > tolerance) {
      std::string p1, p2;
      for (int j = 0; j < new_array.rows(); j++) {
        p1 += " " + std::to_string(new_array[j]);
        p2 += " " + std::to_string(old_array[j]);
      }
      ROS_ERROR("%s New: %s, Old: %s", name, p1.c_str(), p2.c_str());
      exit(0);
      return;
    }
  }
}
void TestTwoFloats(const char* name, float new_float, float old_float, float tolerance) {
  if (fabs(new_float - old_float) > tolerance) {
      ROS_ERROR("%s New: %g, Old: %g", name, new_float, old_float);
      exit(0);
  }
}


void PMCSim::Step(void) {
  b1.Step(impeller_cmd_[0], servo_cmd_[0], omega_, voltage_);
  b2.Step(impeller_cmd_[1], servo_cmd_[1], omega_, voltage_);

  for (int i = 0; i < 2; i++) {
    gnc_.states_[i].impeller_cmd = impeller_cmd_[i];
    for (int j = 0; j < 6; j++)
      gnc_.states_[i].servo_cmd[j] = servo_cmd_[i][j];
  }
  gnc_.Step();

  Blower b[2] = {b1, b2};
  for (int i = 0; i < 2; i++) {
    TestTwoVectors("force", b[i].Force(), gnc_.states_[i].force_B, 0.001);
    TestTwoVectors("torque", b[i].Torque(), gnc_.states_[i].torque_B, 0.001);
    TestTwoVectors("servo current", b[i].ServoCurrent(), gnc_.states_[i].servo_current, 0.001);
    TestTwoFloats("current", b[i].ImpellerCurrent(), gnc_.states_[i].impeller_current, 0.001);
    TestTwoVectors("theta", b[i].NozzlesTheta(), gnc_.states_[i].nozzle_theta, 0.001);
    TestTwoFloats("motor speed", b[i].MotorSpeed(), gnc_.states_[i].motor_speed, 0.001);
  }
}

void PMCSim::SetAngularVelocity(float x, float y, float z) {
  omega_ << x, y, z;
  gnc_.SetAngularVelocity(x, y, z);
}

void PMCSim::SetBatteryVoltage(float voltage) {
  voltage_ = voltage;
  gnc_.SetBatteryVoltage(voltage);
}
}  // namespace pmc_sim

