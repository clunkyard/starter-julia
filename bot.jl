# Narrow the scan around a contact and lead shots from recent sightings.
# Clunkyard guide: https://clunkyard.com/sdk/julia
using ClunkyardSDK
const cy=ClunkyardSDK
const SEARCH_ARC=40.0  # degrees: sweeps the circle in half a second
const HOLD_WIDTH=60.0  # units across at the target: it stays in the beam from tick to tick
const MIN_ARC=6.0      # degrees: the narrowest the beam holds
const WINDOW=12        # ticks of sightings the aim averages
const SIGHTINGS=8      # narrow sightings in the window before a shot
const LOOK_EVERY=40    # ticks between looks around the target, for a nearer robot
const REACQUIRE=10     # ticks to look again where a lost target was
const free=Ref(false)
const dodges,dodge_end,slot=Ref(0),Ref(0),Ref(0)
steer(bot,towards,current)=bot.angles.clamp(bot.angles.angle_diff(towards,current)/45)

function start(bot)
    free[]=bot.match.is_3d && bot.arena.gravity.z==0
end

# A straight line through the narrowest of the last WINDOW ticks' sightings:
# where the robot is now, how fast it moves, and how many sightings it rests
# on. Each sighting is off by its bearing_error; the line averages that out.
function fit(bot,slot,tick)
    # Plain loops: the hosted runtime interprets this file.
    recent,best=[],Inf
    for s in bot.tracker.history(slot)
        if s.tick>tick-WINDOW
            push!(recent,s)
            best=min(best,s.bearing_error)
        end
    end
    length(recent)<2 && return nothing
    seen=[]
    for s in recent
        s.bearing_error<=1.5*best && push!(seen,s)
    end
    n=length(seen)
    n<2 && return nothing
    t0=0.0
    for s in seen; t0+=s.tick/n; end
    spread=0.0
    for s in seen; spread+=(s.tick-t0)^2; end
    position,velocity=Float64[],Float64[]
    for axis in 1:3
        mean,slope=0.0,0.0
        for s in seen; mean+=component(s.position,axis)/n; end
        for s in seen; slope+=(s.tick-t0)*(component(s.position,axis)-mean); end
        slope/=spread
        push!(position,mean+slope*(tick-t0))
        push!(velocity,slope*bot.match.tick_rate)
    end
    (cy.Vec3(position...),cy.Vec3(velocity...),n)
end
component(v,axis)=axis==1 ? v.x : axis==2 ? v.y : v.z

function turn(bot)
    me,gun,scanner,tick=bot.me,bot.gun,bot.scanner,bot.clock.tick
    position=me.position
    bot.drive.forces(brake=0.2)
    target,cue=bot.sensors.nearest,bot.tracker.cue
    last_seen=slot[]!=0 ? bot.tracker.last_seen(slot[]) : nothing
    if target!==nothing
        slot[]=target.slot
        throttle=target.range>350 ? 0.7 : target.range<200 ? -0.4 : 0.2
        bot.drive.set(throttle=throttle,turn=steer(bot,target.bearing,me.heading),pitch_turn=steer(bot,target.elevation,me.pitch))
        # Halve the beam each tick until it is HOLD_WIDTH across at the target:
        # narrow enough for a precise bearing, wide enough to keep it in.
        hold=max(MIN_ARC,rad2deg(2*atan(HOLD_WIDTH/2,target.range)))
        # In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
        # widen it for a tick: a nearer robot it finds becomes the target.
        arc=tick%LOOK_EVERY==0 ? SEARCH_ARC : max(hold,min(SEARCH_ARC,scanner.last.arc/2))
        scanner.set(heading=target.bearing,arc=arc,pitch=target.elevation)
        power=gun.power_for_flight_time(target.range,1)
        # An exact contact: gun.lead is the intercept if the target holds
        # course. A noisy one: lead the fitted line, and fire only once it
        # rests on enough sightings.
        ready=true
        if target.bearing_error>0
            line=fit(bot,target.slot,tick)
            aim=line===nothing ? nothing : gun.lead_point(line[1],line[2],power)
            ready=line!==nothing && line[3]>=SIGHTINGS
        else
            aim=gun.lead(target,power)
        end
        heading,pitch=aim===nothing ? (target.bearing,target.elevation) : (aim.heading,aim.pitch)
        gun.aim(heading=heading,pitch=pitch)
        now=gun.aim_now
        if ready && abs(bot.angles.angle_diff(heading,now.heading))<3 && abs(pitch-now.pitch)<3 && gun.can_fire(power,10)
            gun.fire(power=power)
        end
        bot.status.set(text=ready ? "TRACK" : "NARROW",value=target.range)
    elseif last_seen!==nothing && tick-last_seen<=REACQUIRE
        # Lost it: look again where it was last seen, widening the beam, before searching.
        where=bot.tracker.position(slot[])
        bearing=bot.angles.bearing_to(position,where)
        elevation=free[] ? bot.angles.elevation_to(position,where) : 0.0
        bot.drive.set(throttle=0.5,turn=steer(bot,bearing,me.heading),pitch_turn=steer(bot,elevation,me.pitch))
        scanner.set(heading=bearing,arc=min(SEARCH_ARC,scanner.last.arc*2),pitch=elevation)
        bot.status.set(text="REACQUIRE")
    elseif cue!==nothing
        bearing,distance=bot.angles.bearing_to(position,cue.position),bot.angles.distance(position,cue.position)
        elevation=free[] ? bot.angles.elevation_to(position,cue.position) : 0.0
        error=min(cue.bearing_error,30)
        across=(tick%5-2)/2*error
        up=free[] ? (div(tick,5)%3-1)*error/2 : 0.0
        bot.drive.set(throttle=0.7,turn=steer(bot,bearing,me.heading),pitch_turn=steer(bot,elevation,me.pitch))
        scanner.set(heading=bearing+across,arc=scanner.arc_for_reach(distance+100),pitch=elevation+up)
        bot.status.set(text="CUE",value=distance)
    else
        bot.drive.set(throttle=0.5,turn=0.33,pitch_turn=steer(bot,0,me.pitch))
        pitch=free[] ? (div(tick,10)%3-1)*35.0 : 0.0
        scanner.set(heading=bot.angles.normalize(scanner.last.heading+36),arc=SEARCH_ARC,pitch=pitch)
        bot.status.set(text="SEARCH")
    end
    # A simple dodge: vary it, or predict the shooter.
    if tick>=dodge_end[] && any(c->c.source==:muzzle,bot.sensors.cues)
        dodges[]+=1
        dodge_end[]=tick+10
    end
    if tick<dodge_end[]
        # Off the line of fire: left, then right next time (turn is positive left, strafe right).
        left=isodd(dodges[])
        if bot.match.control==:forces
            bot.drive.set(throttle=0.5)
            bot.drive.forces(strafe=left ? -1.0 : 1.0)
        else
            bot.drive.set(throttle=1.0,turn=left ? 1.0 : -1.0)
        end
        bot.status.set(text=left ? "dodge:left" : "dodge:right",value=dodges[])
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
end

cy.run(cy.Robot(on_start=start,on_turn=turn))
