local announced = false

function update()
    if not announced then
        gcs:send_text(0, "JarvisH743 custom board, provded and developed by TechUnity.")
        announced = true
    end
    return update, 1000
end

return update()
