const START_CMD = JSON.json(Dict("action" => "start_experiment")) * "\n"
const STOP_CMD = JSON.json(Dict("action" => "stop_experiment")) * "\n"
const GET_TIME_CMD = JSON.json(Dict("action" => "get_time_elapsed")) * "\n"
const GET_ROLLOUT_DATA = JSON.json(Dict("action" => "get_rollout_data")) * "\n"

function open_tb_connections()
    ip = "192.168.1.223"
    feedback_ports = [50010, 50020, 50030]
    rollout_ports = [50012, 50022, 50032]
    ts_ports = [50013, 50023, 50033]
    coeffs_x_ports = [50014, 50024, 50034]
    coeffs_y_ports = [50015, 50025, 50035]
    timing_port = 50011

    timing = open_connection(ip, timing_port)
    tbs = Vector{TurtlebotConnection}()
    for i in 1:3
        push!(
            tbs, 
            TurtlebotConnection(
                open_connection(ip, feedback_ports[i]),
                open_connection(ip, rollout_ports[i]),
                open_connection(ip, ts_ports[i]),
                open_connection(ip, coeffs_x_ports[i]),
                open_connection(ip, coeffs_y_ports[i])
            )
        )
    end
    return Connections(tbs, timing)
end

# Gets the pose and twist and converts it to a state consisting of [x,y,v,θ].
function state(tb::TurtlebotConnection)
    feedback_data = receive_feedback_data(tb.feedback)
    x = feedback_data.position[1]
    y = feedback_data.position[2]

    rot_mat = QuatRotation(
        feedback_data.orientation[1],
        feedback_data.orientation[2],
        feedback_data.orientation[3],
        feedback_data.orientation[4]
    )
    heading = rot_mat*[1,0,0]
    θ = atan(heading[2],heading[1])

    v = norm(feedback_data.linear_vel[1:2])
    if abs(atan(feedback_data.linear_vel[2],feedback_data.linear_vel[1]) - θ) > π/2
        v = -v
    end

    return [x,y,v,θ]
end

function start_robots(connections::Connections)
    @info "Starting robots..."
    send(connections.timing, START_CMD)
end

function stop_robots(connections::Connections)
    @info "Stopping robots"
    send(connections.timing, STOP_CMD)
end

"""Sends the GET_TIME_CMD to the timing connection, waits for a response, then
extracts the elapsed_time from the response."""
function time_elapsed(connections::Connections)
    payload = send_receive(connections.timing, GET_TIME_CMD)
    data = JSON.parse(String(payload))
    return data["elapsed_time"]
end

function send_spline_params(tb::TurtlebotConnection, spl::Spline)
    @info "Sending spline parameters"
    x_coeffs = vec(spl.all_x_coeffs')
    y_coeffs = vec(spl.all_y_coeffs')
    
    send(tb.ts, JSON.json(Dict("array" => spl.ts)) * "\n")
    send(tb.coeffs_x, JSON.json(Dict("array" => x_coeffs)) * "\n")
    send(tb.coeffs_y, JSON.json(Dict("array" => y_coeffs)) * "\n")
end

function send_splines(connections::Connections, splines::Vector{Spline})
    for (i, spl) in enumerate(splines)
        send_spline_params(connections.tbs[i], spl)
    end
end

function close_tb_connections(connections::Connections)
    close_connection(connections.timing)
    for tb in connections.tbs
        close_connection(tb.feedback)
        close_connection(tb.rollout)
        close_connection(tb.ts)
        close_connection(tb.coeffs_x)
        close_connection(tb.coeffs_y)
    end
end