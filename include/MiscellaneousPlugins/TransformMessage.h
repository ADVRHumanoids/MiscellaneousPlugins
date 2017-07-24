 /*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Enrico Mingo
 * email:  arturo.laurenzi@iit.it, enrico.mingo@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __MISCPLUGINS_TRANSFORM_MESSAGE_H__
#define __MISCPLUGINS_TRANSFORM_MESSAGE_H__

#include <XBotCore-interfaces/XDomainCommunication.h>

namespace XBot {

    struct TransformMessage {

        Command child_frame;
        Command parent_frame;
        Eigen::Affine3d pose;

    };

}

#endif