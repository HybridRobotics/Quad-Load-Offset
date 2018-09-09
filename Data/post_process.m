figure;
hold on
errorbar(freq,old,oldUncertainty,'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
errorbar(freq,new,newUncertainty,'-o','MarkerSize',10,...
    'MarkerEdgeColor','blue','MarkerFaceColor','blue')
set(gca,'YScale','log')
xlabel('f (hz)')
ylabel('e_{x_L} (m)')
grid on
legend('Traditional geometric control design','Improved geometric control design')