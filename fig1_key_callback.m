function fig1_key_callback(hObject, event, handles)
   v = handles.cart_v;
   vstep = handles.cart_vstep;
   switch event.Key
       case 'leftarrow'
           v = v-vstep;
       case 'rightarrow'
           v = v+vstep;
       case 'downarrow'
           v = 0.1*v;
           
       otherwise
           %do nothing
           
   end
   handles.cart_v = v;
   handles.is_called = true;
   guidata(hObject,handles); %update the handles of the gui object
end
