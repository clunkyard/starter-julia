# A robot on the core binding supplied by the arena.
# The core owns protocol parsing, tracking, geometry and act serialization.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
using ClunkyardSDK
const cy=ClunkyardSDK
const free=Ref(false)
steer(bot,towards,current)=bot.angles.clamp(bot.angles.angle_diff(towards,current)/45)

function start(bot)
    free[]=bot.match.is_3d && bot.arena.gravity.z==0
end
function turn(bot)
    me,gun,scanner=bot.me,bot.gun,bot.scanner
    position=me.position
    bot.drive.forces(brake=0.2)
    target,cue=bot.sensors.nearest,bot.tracker.cue
    if target!==nothing
        throttle=target.range>350 ? 0.7 : target.range<200 ? -0.4 : 0.2
        bot.drive.set(throttle=throttle,turn=steer(bot,target.bearing,me.heading),pitch_turn=steer(bot,target.elevation,me.pitch))
        scanner.set(heading=target.bearing,arc=20,pitch=target.elevation)
        power=gun.power_for_flight_time(target.range,1)
        aim=gun.lead(target,power)
        heading,pitch=aim===nothing ? (target.bearing,target.elevation) : (aim.heading,aim.pitch)
        gun.aim(heading=heading,pitch=pitch)
        now=gun.aim_now
        if abs(bot.angles.angle_diff(heading,now.heading))<3 && abs(pitch-now.pitch)<3 && gun.can_fire(power,10)
            gun.fire(power=power)
        end
        bot.status.set(text="TRACK",value=target.range)
    elseif cue!==nothing
        bearing,distance=bot.angles.bearing_to(position,cue.position),bot.angles.distance(position,cue.position)
        elevation=free[] ? bot.angles.elevation_to(position,cue.position) : 0.0
        error,tick=min(cue.bearing_error,30),bot.clock.tick
        across=(tick%5-2)/2*error
        up=free[] ? (div(tick,5)%3-1)*error/2 : 0.0
        bot.drive.set(throttle=0.7,turn=steer(bot,bearing,me.heading),pitch_turn=steer(bot,elevation,me.pitch))
        scanner.set(heading=bearing+across,arc=scanner.arc_for_reach(distance+100),pitch=elevation+up)
        bot.status.set(text="CUE",value=distance)
    else
        bot.drive.set(throttle=0.5,turn=0.33,pitch_turn=steer(bot,0,me.pitch))
        pitch=free[] ? (div(bot.clock.tick,10)%3-1)*35.0 : 0.0
        scanner.set(heading=bot.angles.normalize(scanner.last.heading+36),arc=40,pitch=pitch)
        bot.status.set(text="SEARCH")
    end
    bot.battery.charge<bot.battery.capacity/4 && bot.drive.set(throttle=something(bot.drive.get("throttle"),0.0)*0.5)
    storm,zone=bot.nav.storm,bot.nav.zone
    if storm!==nothing && something(storm.margin(position),-1e9)> -60
        away=bot.angles.bearing_to(storm.center,position)
        bot.drive.set(throttle=0.8,turn=bot.angles.clamp(bot.angles.angle_diff(away,me.heading)/30))
    end
    if zone!==nothing && something(zone.margin(position),1e9)<60
        climb=zone.shape==:sphere ? bot.angles.elevation_to(position,zone.center) : 0.0
        bot.drive.set(throttle=0.8,turn=bot.angles.clamp(bot.angles.angle_diff(bot.angles.bearing_to(position,zone.center),me.heading)/30),pitch_turn=bot.angles.clamp((climb-me.pitch)/30))
    end
    # The arena's rate calibration only; safe to delete.
    work=bot.raw.number("state.calibration.work")
    work!==nothing && work>0 && bot.status.set(text="WORK",value=calibration_work(Int(work),bot.clock.tick))
end

# The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit xorshift
# and a square-root sum.
function calibration_work(work, tick)
    x, sum = UInt64(88172645463325252) ⊻ UInt64(tick), 0.0
    for _ in 1:work
        x ⊻= x << 13
        x ⊻= x >> 7
        x ⊻= x << 17
        sum += sqrt(Float64(x >> 11))
    end
    return (floor(Int, sum) + Int(x >> 40)) % 1000000
end

cy.run(cy.Robot(on_start=start,on_turn=turn))
