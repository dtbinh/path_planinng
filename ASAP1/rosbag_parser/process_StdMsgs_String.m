function data = process_StdMsgs_String(data_raw, time)

%time data
data = cell(length(data_raw), 2);
for i=1:length(data_raw)
    data{i,1} = time(i);
    data{i,2} = data_raw{i}.Data;
end

end