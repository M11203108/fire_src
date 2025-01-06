
class Spray2FirePoint:
    def __init__(self):
        pass

    def spray_move(self, angle, distance):
        """
        控制噴頭移動
        """
        print(f"噴頭移動: 角度={angle:.2f}°, 距離={distance:.2f}m")
        return True
    
    def spray(self):
        """
        噴火
        """
        # 噴火
        # ...
        print("噴火")
        return True
    
    def main(self, angle, distance):

        # 控制噴頭移動
        self.spray_move(angle, distance)
        
        # 噴火
        self.spray()
        
        return True
